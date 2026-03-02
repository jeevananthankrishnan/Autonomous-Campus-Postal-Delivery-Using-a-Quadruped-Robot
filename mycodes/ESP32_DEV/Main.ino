#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WMM_Tinier.h>
#include <array>

// Project Headers
#include "Ultrasonic.h"
#include "GPS.h"
#include "IMU.h"
#include "Lidar.h"
#include "Display.h"
#include "SensorData.h"
#include "Geolocation.h"
#include "Navigation.h"
#include "Controller.h" // Integrated Controller

// ==================================================================================
// CONFIGURATION
// ==================================================================================
#define DEV_LED_PIN 2
#define MAGNETIC_DECLINATION 4.2f
#define YAW_NORTH_OFFSET 100.0f

// --- BLIND MODE 1 CONFIGURATION (After Point 1) ---
#define HEADING_BM1_N   20.0f    // "North 20 degree"
#define HEADING_BM1_E   110.0f   // Turn to 110 degree
#define DIST_BM1_MIN_N  3.0f     // Move at least 3m North before checking wall
#define DIST_BM1_LEG_E  17.0f    // Move 17m in the 110 deg direction
#define WALL_STOP_DIST  150       // Stop when wall is 80cm away 

// --- BLIND MODE 2 CONFIGURATION (After Point 5) ---
#define HEADING_BM2_W   215.0f   // Align West (280 degree)
#define DIST_BM2_LEG    10.0f    // Move forward 10 meters

// OBSTACLE AVOIDANCE
#define OBSTACLE_DIST_THRESH 90  // 50 cm threshold for Front
#define SIDE_OBSTACLE_THRESH 40  // 30 cm threshold for Side (New)

// WMM DATE
const uint8_t WMM_YEAR = 25;
const uint8_t WMM_MONTH = 1;
const uint8_t WMM_DAY = 1;

// ==================================================================================
// WAYPOINT MAPS
// ==================================================================================
struct WayPoints {
    static constexpr double MONUMENT_LAT = 48.829950;
    static constexpr double MONUMENT_LON = 12.955072;
    static constexpr double POINT1_LAT = 48.830067;  // Target 1 (Start Point)
    static constexpr double POINT1_LON = 12.954973;
    static constexpr double POINT2_LAT = 48.829910;  // Target 2 
    static constexpr double POINT2_LON = 12.955085; 
    static constexpr double POINT3_LAT = 48.829765;  // Target 3
    static constexpr double POINT3_LON = 12.954965;
    static constexpr double POINT4_LAT = 48.829470;  // Target 4
    static constexpr double POINT4_LON = 12.954710;
    static constexpr double POINT5_LAT = 48.829572;  // Target 5
    static constexpr double POINT5_LON = 12.954370;
};

// ==== PATH DEFINITION ====
struct GeoPoint { double lat; double lon; };

GeoPoint gpsPath[] = {
    {WayPoints::POINT1_LAT, WayPoints::POINT1_LON}, 
    {WayPoints::POINT2_LAT, WayPoints::POINT2_LON}, 
    {WayPoints::POINT3_LAT, WayPoints::POINT3_LON}, 
    {WayPoints::POINT4_LAT, WayPoints::POINT4_LON}, 
    {WayPoints::POINT5_LAT, WayPoints::POINT5_LON}, 
};
const int TOTAL_WAYPOINTS = sizeof(gpsPath) / sizeof(gpsPath[0]);

const int IDX_PT1 = 0;
const int IDX_PT3 = 2;
const int IDX_PT5 = 4; 

struct LocalPoint { float n; float e; };
LocalPoint localPath[TOTAL_WAYPOINTS];

// ==================================================================================
// MULTI-CORE & SYNC OBJECTS
// ==================================================================================
TaskHandle_t Task0;
SemaphoreHandle_t dataMutex; 

struct SharedData {
    // Inputs
    uint16_t us_front, us_left, us_right, us_back, us_incline;
    float yaw;
    double lat, lon;
    bool gps_valid;
    bool lidar_valid;
    uint16_t lidar_dist;
    
    // Outputs
    SystemState state;
    NavCommand cmd;
    float dist_to_target;
    int current_wp_index;
    bool is_mission_active;
} sharedData;

// ==================================================================================
// MISSION STATE MACHINE
// ==================================================================================
enum MissionStep {
    STEP_IDLE,
    STEP_NAV_TO_POINT1,  
    STEP_BM1_ALIGN_NORTH,    
    STEP_BM1_FWD_SEARCH,     
    STEP_BM1_APPROACH_WALL,  
    STEP_BM1_ALIGN_EAST,     
    STEP_BM1_PAUSED,         
    STEP_BM1_FWD_TO_PT2,     
    STEP_RESUME_GPS_TO_PT5,  
    STEP_BM2_ALIGN_WEST,     
    STEP_BM2_FWD_TO_PT6,    
    STEP_RETURN_NAV, 
    STEP_DONE
};

enum MissionType {
    MISSION_FULL,
    MISSION_POST_ONLY,
    MISSION_RETURN 
};

MissionStep currentMissionStep = STEP_IDLE;
MissionType currentMissionType = MISSION_FULL; 

double legStartLat = 0.0;
double legStartLon = 0.0;
float distTraveledLeg = 0.0; 

// ==================================================================================
// GLOBAL OBJECTS
// ==================================================================================
UltrasonicSensor usFront(32, 25);
UltrasonicSensor usBack(5, 14);
UltrasonicSensor usLeft(4, 18);
UltrasonicSensor usRight(13, 19);
UltrasonicSensor usInclined(23, 26);
GPSModule gps(Serial2);
IMU imu2(0x28);
LidarLD20 lidar(Serial1);
Display display;
WMM_Tinier wmm;
Geolocation geo(0.0, 0.0);
Navigation navigator;

struct_message espNowData;
CommandMessage receivedCmd;

// --- GLOBAL FLAGS FOR STOP/RESUME ---
volatile bool emergencyStopTriggered = false;
bool isMissionPaused = false; 
volatile bool triggerInitRoutine = false; // <--- NEW FLAG
// ----------------------------------------

float magneticDeclination = MAGNETIC_DECLINATION;
const float GPS_FILTER_ALPHA = 0.2f; 
double filteredLat = 0.0, filteredLon = 0.0;
bool isFirstGpsReading = true;

// Timings
unsigned long lastBasicSendTime = 0;
unsigned long lastDisplayTime = 0;
unsigned long lastLidarScanStartTime = 0;
unsigned long lastLidarPacketSendTime = 0;
const long BASIC_SEND_INTERVAL = 200;
const long DISPLAY_INTERVAL = 100;
const long LIDAR_SCAN_INTERVAL = 80;
const long LIDAR_PACKET_INTERVAL = 2;

// Lidar State
uint16_t totalLidarPoints = 0;
uint8_t totalLidarPackets = 0;
uint8_t currentLidarPacketIndex = 0;
bool isSendingLidar = false;
uint8_t espCamAddress[] = {0xFC, 0xB4, 0x67, 0xF0, 0x8A, 0x28}; 

SystemState currentState = STATE_INITIALIZING;
NavCommand currentNavCommand = NAV_WAIT;

// ==================================================================================
// PROTOTYPES
// ==================================================================================
void Task0Code(void * pvParameters);
void sendBasicSensorData();
void prepareLidarScan();
void sendNextLidarPacket();
void convertPathToLocal();
void OnDataSent(const wifi_tx_info_t *mac, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info_t *mac, const uint8_t *incomingData, int len);
void logSensorData(); 
uint16_t normalizeUS(int val) { return (val < 0 || val > 65000) ? 999 : val; }

// ==================================================================================
// SETUP
// ==================================================================================
void setup() {
    Serial.begin(115200);
    Wire.begin(); 

    // Initialize Motor/Controller Hardware
    if (!initDAC()) Serial.println("DAC Init Failed");
    if (!initExpMod()) Serial.println("Expander Init Failed");
    
    // Create Mutex
    dataMutex = xSemaphoreCreateMutex();
    
    pinMode(DEV_LED_PIN, OUTPUT);
    if (!display.begin()) Serial.println("SSD1306 failed");

    usFront.begin(); usBack.begin(); usLeft.begin(); usRight.begin(); usInclined.begin();
    if (!imu2.begin()) { Serial.println("BNO055 Failed"); while(1); }
    wmm.begin();
    gps.begin(16, 17);
    lidar.begin(27, 33);

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) return;
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, espCamAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    convertPathToLocal();
    sharedData.state = STATE_WAITING_FOR_GPS;
    sharedData.cmd = NAV_WAIT;
    sharedData.is_mission_active = false;
    sharedData.current_wp_index = 0;

    // Start Core 0 Task
    xTaskCreatePinnedToCore(Task0Code, "LogicTask", 10000, NULL, 1, &Task0, 0);
    Serial.println("Dual Core System Ready.");
}

// ==================================================================================
// MAIN LOOP (CORE 1) - FAST SENSORS
// ==================================================================================
void loop() {
    unsigned long now = millis();
    lidar.update();
    gps.update();

    // =========================================================
    // MOTOR CONTROL LOGIC (CRITICAL FOR MOVEMENT)
    // =========================================================
    if (triggerInitRoutine) {
        stop(); // Ensure stopped before starting
        Serial.println(">>> EXECUTING INITIALIZATION ROUTINE...");
        
        // This blocks the loop for ~30 seconds, which is fine for Init
        initialcheckuproutine(); 
        
        triggerInitRoutine = false; // Reset flag
        Serial.println(">>> INITIALIZATION COMPLETE.");
    }
    // ----------------------------

    // 1. If Hardware Stop Triggered OR Paused OR Mission Inactive -> STOP
    else if (emergencyStopTriggered || isMissionPaused || !sharedData.is_mission_active) {
        stop(); 
    }
    // 2. If Mission is Active and NOT Paused -> MOVE
    else {
        // Read the command decided by Task0
        NavCommand moveCmd = NAV_WAIT;
        if (xSemaphoreTake(dataMutex, (TickType_t) 5) == pdTRUE) {
            moveCmd = sharedData.cmd;
            xSemaphoreGive(dataMutex);
        }

        // Execute Motor Command from Controller.h
        switch (moveCmd) {
            case NAV_FORWARD:    forward(28); break; // Adjust speed (28) as needed
            case NAV_TURN_LEFT:  rotateLeft(28); break; 
            case NAV_TURN_RIGHT: rotateRight(28); break;
            case NAV_STRAFE_LEFT: stepLeft(28); break; 
            case NAV_STRAFE_RIGHT: stepRight(28); break;
            case NAV_WAIT:       stop(); break;
            default:             stop(); break;
        }
    }
    // =========================================================

    if (isSendingLidar) {
        if (now - lastLidarPacketSendTime >= LIDAR_PACKET_INTERVAL) {
            lastLidarPacketSendTime = now;
            sendNextLidarPacket();
        }
    } else if (now - lastLidarScanStartTime >= LIDAR_SCAN_INTERVAL) {
        lastLidarScanStartTime = now;
        prepareLidarScan();
    }

    if (now - lastBasicSendTime >= BASIC_SEND_INTERVAL) {
        sendBasicSensorData();
    }
}

// ==================================================================================
// CORE 0 TASK - MISSION LOGIC & DISPLAY
// ==================================================================================
void Task0Code(void * pvParameters) {
    // --- CHANGE: Buffer for Thread-Safe Lidar Data ---
    static LidarPoint localLidarData[360];

    for(;;) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        NavCommand computedCmd = NAV_WAIT;
        SharedData localCopy;
        
        if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
            localCopy = sharedData;
            xSemaphoreGive(dataMutex);
        } else {
            continue;
        }

        // --- CHANGE: Take Lidar Snapshot ---
        // Copies data safely from Core 1 to Core 0 to prevent Race Conditions
        lidar.getSafePoints(localLidarData);

        // --- MISSION LOGIC START ---
        if (localCopy.is_mission_active) {
            
            // 1. CHECK PAUSE STATE
            if (isMissionPaused) {
                // If paused via Web, set specific command for "W_ORD"
                computedCmd = NAV_STOPPED; 
            } 
            else { 
                // >>>> ELSE START: ALL NAVIGATION LOGIC GOES INSIDE HERE <<<<

                // --- DISTANCE CALCULATION ---
                if (currentMissionStep == STEP_NAV_TO_POINT1 || 
                    currentMissionStep == STEP_RESUME_GPS_TO_PT5 ||
                    currentMissionStep == STEP_RETURN_NAV) {
                    
                    int idx = localCopy.current_wp_index;
                    if(idx < 0) idx = 0;
                    if(idx >= TOTAL_WAYPOINTS) idx = TOTAL_WAYPOINTS - 1;

                    float tN = localPath[idx].n;
                    float tE = localPath[idx].e;
                    float cN, cE;
                    geo.toLocalCoordinates(localCopy.lat, localCopy.lon, cN, cE);
                    distTraveledLeg = geo.getLocalDistance(cN, cE, tN, tE);
                } 
                else if (legStartLat != 0.0 && localCopy.gps_valid) {
                    distTraveledLeg = geo.haversineDistance(legStartLat, legStartLon, localCopy.lat, localCopy.lon);
                }

                switch(currentMissionStep) {
                    // PHASE 1: GPS TO POINT 1
                    case STEP_NAV_TO_POINT1:
                    {
                        float targetN = localPath[IDX_PT1].n; 
                        float targetE = localPath[IDX_PT1].e;
                        float cN, cE;
                        geo.toLocalCoordinates(localCopy.lat, localCopy.lon, cN, cE);
                        computedCmd = navigator.computeLocalPathCommand(cN, cE, targetN, targetE, localCopy.yaw);
                        
                        if (distTraveledLeg < 3.0f) {
                            Serial.println("Core0: Reached Point 1. Switching to BLIND MODE 1.");
                            currentMissionStep = STEP_BM1_ALIGN_NORTH;
                        }
                        break;
                    }

                    // PHASE 2: BLIND MODE 1
                    case STEP_BM1_ALIGN_NORTH:
                        computedCmd = navigator.computeHeadingCommand(HEADING_BM1_N, localCopy.yaw, false);
                        if (computedCmd == NAV_WAIT) { 
                            legStartLat = localCopy.lat; 
                            legStartLon = localCopy.lon;
                            currentMissionStep = STEP_BM1_FWD_SEARCH;
                        }
                        break;

                    case STEP_BM1_FWD_SEARCH:
                        computedCmd = navigator.computeHeadingCommand(HEADING_BM1_N, localCopy.yaw, true);
                        // Check Front US
                        if (localCopy.us_front > 0 && localCopy.us_front < 150) {
                            Serial.println("Core0: Wall detected (US). Approaching...");
                            currentMissionStep = STEP_BM1_APPROACH_WALL;
                        }
                        // Check Lidar after min distance
                        else if (distTraveledLeg > DIST_BM1_MIN_N && localCopy.lidar_valid && localCopy.lidar_dist > 0 && localCopy.lidar_dist < 1500) {
                             Serial.println("Core0: Wall detected (Lidar). Approaching...");
                             currentMissionStep = STEP_BM1_APPROACH_WALL;
                        }
                        break;

                    case STEP_BM1_APPROACH_WALL:
                        // Keep moving forward until WALL_STOP_DIST is met
                        if (localCopy.us_front > WALL_STOP_DIST || localCopy.us_front == 0) {
                             computedCmd = navigator.computeHeadingCommand(HEADING_BM1_N, localCopy.yaw, true);
                        } else {
                            computedCmd = NAV_WAIT;
                            Serial.println("Core0: Wall Reached. Aligning East.");
                            currentMissionStep = STEP_BM1_ALIGN_EAST;
                        }
                        break;

                    case STEP_BM1_ALIGN_EAST:
                        computedCmd = navigator.computeHeadingCommand(HEADING_BM1_E, localCopy.yaw, false);
                        if (computedCmd == NAV_WAIT) {
                            legStartLat = localCopy.lat; 
                            legStartLon = localCopy.lon;
                            
                            if (currentMissionType == MISSION_POST_ONLY) {
                                Serial.println("Core0: POST Mission Reached Limit. Pausing.");
                                currentMissionStep = STEP_BM1_PAUSED;
                            } else {
                                currentMissionStep = STEP_BM1_FWD_TO_PT2;
                            }
                        }
                        break;

                    case STEP_BM1_PAUSED:
                        computedCmd = NAV_WAIT;
                        if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
                            sharedData.state = STATE_READY;
                            sharedData.is_mission_active = false; 
                            xSemaphoreGive(dataMutex);
                        }
                        break;

                    case STEP_BM1_FWD_TO_PT2:
                        computedCmd = navigator.computeHeadingCommand(HEADING_BM1_E, localCopy.yaw, true);
                        if (distTraveledLeg >= DIST_BM1_LEG_E) {
                            currentMissionStep = STEP_RESUME_GPS_TO_PT5;
                            if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
                                sharedData.current_wp_index = IDX_PT3; 
                                xSemaphoreGive(dataMutex);
                            }
                            Serial.println("Core0: Blind 1 Done. Resuming GPS to Point 3.");
                        }
                        break;

                    // PHASE 3: GPS NAVIGATION
                    case STEP_RESUME_GPS_TO_PT5:
                    {
                        int idx = localCopy.current_wp_index;
                        float targetN = localPath[idx].n;
                        float targetE = localPath[idx].e;
                        float cN, cE;
                        geo.toLocalCoordinates(localCopy.lat, localCopy.lon, cN, cE);
                        computedCmd = navigator.computeLocalPathCommand(cN, cE, targetN, targetE, localCopy.yaw);
                        
                        if (distTraveledLeg < 2.0f) {
                            if (idx >= IDX_PT5) {
                                Serial.println("Core0: Reached Point 5. Switching to BLIND MODE 2.");
                                legStartLat = localCopy.lat; 
                                legStartLon = localCopy.lon;
                                distTraveledLeg = 0;
                                currentMissionStep = STEP_BM2_ALIGN_WEST;
                            } else {
                                if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
                                    sharedData.current_wp_index++;
                                    xSemaphoreGive(dataMutex);
                                }
                            }
                        }
                        break;
                    }
                    
                    // PHASE 4: BLIND MODE 2
                    case STEP_BM2_ALIGN_WEST:
                        computedCmd = navigator.computeHeadingCommand(HEADING_BM2_W, localCopy.yaw, false);
                        if (computedCmd == NAV_WAIT) {
                            legStartLat = localCopy.lat; 
                            legStartLon = localCopy.lon;
                            currentMissionStep = STEP_BM2_FWD_TO_PT6;
                        }
                        break;

                    case STEP_BM2_FWD_TO_PT6:
                        computedCmd = navigator.computeHeadingCommand(HEADING_BM2_W, localCopy.yaw, true);
                        if (distTraveledLeg >= DIST_BM2_LEG) {
                            Serial.println("Core0: Blind 2 Done. Mission Complete.");
                            computedCmd = NAV_WAIT;
                            currentMissionStep = STEP_DONE;
                            
                            if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
                                sharedData.state = STATE_REACHED;
                                xSemaphoreGive(dataMutex);
                            }
                        }
                        break;

                    // --- STEP: RETURN TO START (WAYPOINT BY WAYPOINT) ---
                    case STEP_RETURN_NAV:
                    {
                        int idx = localCopy.current_wp_index;
                        
                        // Get target coordinates for current waypoint
                        float targetN = localPath[idx].n;
                        float targetE = localPath[idx].e;
                        
                        // Get current position in local coordinates
                        float cN, cE;
                        geo.toLocalCoordinates(localCopy.lat, localCopy.lon, cN, cE);
                        
                        // Compute Navigation Command
                        computedCmd = navigator.computeLocalPathCommand(cN, cE, targetN, targetE, localCopy.yaw);
                        
                        // Check if we reached the waypoint (tolerance 2.0m)
                        if (distTraveledLeg < 2.0f) {
                            if (idx == 0) {
                                // If index is 0, we have reached the Start Point (Point 1)
                                Serial.println("Core0: Returned to Start Point via Waypoints.");
                                computedCmd = NAV_WAIT;
                                currentMissionStep = STEP_DONE;
                                if (xSemaphoreTake(dataMutex, (TickType_t)10)) {
                                    sharedData.state = STATE_REACHED; 
                                    sharedData.is_mission_active = false;
                                    xSemaphoreGive(dataMutex);
                                }
                            } else {
                                // Move to the previous waypoint (backwards direction)
                                if (xSemaphoreTake(dataMutex, (TickType_t)10)) {
                                    sharedData.current_wp_index--; // Decrement index
                                    xSemaphoreGive(dataMutex);
                                }
                                Serial.printf("Core0: Reached Waypoint %d. Backtracking to %d\n", idx, idx-1);
                            }
                        }
                        break;
                    }

                    case STEP_DONE:
                        computedCmd = NAV_WAIT;
                        break;
                    
                    default: 
                        computedCmd = NAV_WAIT; break;
                }

                // ==================================================================
                // OBSTACLE AVOIDANCE OVERRIDE
                // ==================================================================
                if (currentMissionStep != STEP_BM1_APPROACH_WALL && 
                    currentMissionStep != STEP_DONE && 
                    currentMissionStep != STEP_BM1_PAUSED) {
                    
                    // 1. FRONT OBSTACLE (< 50cm) - High Priority
                    if (localCopy.us_front > 0 && localCopy.us_front < OBSTACLE_DIST_THRESH) {
                        if (localCopy.us_left > localCopy.us_right) {
                            computedCmd = NAV_TURN_LEFT;
                        } else {
                            computedCmd = NAV_TURN_RIGHT;
                        }
                    }
                    // 2. SIDE OBSTACLE (< 30cm)
                    else if (localCopy.us_left > 0 && localCopy.us_left < SIDE_OBSTACLE_THRESH) {
                        computedCmd = NAV_STRAFE_RIGHT; 
                    }
                    else if (localCopy.us_right > 0 && localCopy.us_right < SIDE_OBSTACLE_THRESH) {
                        computedCmd = NAV_STRAFE_LEFT; 
                    }
                }
            } // >>>> END OF ELSE BLOCK <<<<
        } 
        else {
            computedCmd = localCopy.cmd; 
        }

        // Write outputs back
        if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
            sharedData.cmd = computedCmd;
            if (currentMissionStep != STEP_IDLE) sharedData.dist_to_target = distTraveledLeg;
            xSemaphoreGive(dataMutex);
        }

        display.update(localCopy.state, computedCmd, localCopy.us_front, localCopy.us_left, localCopy.us_right, localCopy.yaw, localCopy.dist_to_target);
    }
}

// ==================================================================================
// SENSORS & DATA SENDING
// ==================================================================================
void sendBasicSensorData() {
    imu2.updateCalibration();
    
    // READ ALL SENSORS
    uint16_t f = normalizeUS(usFront.readDistance());
    uint16_t l = normalizeUS(usLeft.readDistance());
    uint16_t r = normalizeUS(usRight.readDistance());
    uint16_t b = normalizeUS(usBack.readDistance());
    uint16_t i = normalizeUS(usInclined.readDistance());
    
    // GPS Reading & Filtering
    bool gpsValid = gps.locationValid();
    double lat = 0, lon = 0;
    
    if (gpsValid) {
        double rawLat = gps.getLatitude();
        double rawLon = gps.getLongitude();
        if (isFirstGpsReading) { filteredLat = rawLat; filteredLon = rawLon; isFirstGpsReading = false; }
        else {
            filteredLat = (GPS_FILTER_ALPHA * rawLat) + ((1.0 - GPS_FILTER_ALPHA) * filteredLat);
            filteredLon = (GPS_FILTER_ALPHA * rawLon) + ((1.0 - GPS_FILTER_ALPHA) * filteredLon);
        }
        lat = filteredLat; lon = filteredLon;
        
        float dec = wmm.magneticDeclination(lat, lon, WMM_YEAR, WMM_MONTH, WMM_DAY);
        if (dec > -30 && dec < 30) magneticDeclination = dec;
    }

    // IMU
    float roll, pitch, yaw;
    imu2.readOrientation(roll, pitch, yaw, magneticDeclination);
    yaw -= YAW_NORTH_OFFSET;
    if (yaw < 0) yaw += 360; if (yaw >= 360) yaw -= 360;

    // Lidar
    uint16_t minLidar = 0;
    // Uses Mutex inside Lidar.h
    int idx = lidar.findClosestObstacle(minLidar);
    bool lidarValid = (idx >= 0);

    // Update Shared Data (Lock Mutex)
    if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
        sharedData.us_front = f;
        sharedData.us_left = l;
        sharedData.us_right = r;
        sharedData.us_back = b;
        sharedData.us_incline = i;
        
        sharedData.yaw = yaw;
        sharedData.lat = lat;
        sharedData.lon = lon;
        sharedData.gps_valid = gpsValid;
        sharedData.lidar_valid = lidarValid;
        sharedData.lidar_dist = minLidar;
        
        currentNavCommand = sharedData.cmd;
        xSemaphoreGive(dataMutex);
    }

    // Populate ESP-NOW Packet
    espNowData.us_front = f;
    espNowData.us_left = l;
    espNowData.us_right = r;
    espNowData.us_back = b;
    espNowData.us_incline = i;
    
    espNowData.imu_yaw = yaw;
    espNowData.gps_lat = lat;
    espNowData.gps_lon = lon;
    espNowData.gps_valid = gpsValid;
    espNowData.nav_command = (int8_t)currentNavCommand;
    espNowData.lidar_closest_dist = minLidar;
    espNowData.timestamp = millis();
    espNowData.gps_distance_to_monument = sharedData.dist_to_target; 
    
    // Log & Send
    logSensorData(); 
    esp_now_send(espCamAddress, (uint8_t *)&espNowData, sizeof(espNowData));
    lastBasicSendTime = millis();
}

void logSensorData()
{
    Serial.println("📡 Sensor Data:");
    Serial.printf("  US F:%d L:%d R:%d B:%d\n", espNowData.us_front, espNowData.us_left, espNowData.us_right, espNowData.us_back);
    Serial.printf("  Yaw: %.1f°\n", espNowData.imu_yaw);
    
    if (sharedData.is_mission_active) {
        Serial.printf("  [MISSION] Step: %d | Leg Dist: %.2fm | Target WP Index: %d\n", 
                      currentMissionStep, sharedData.dist_to_target, sharedData.current_wp_index);
    }

    if (espNowData.gps_lat != 0.0) {
        Serial.printf("  GPS: %.6f, %.6f\n", espNowData.gps_lat, espNowData.gps_lon);
    } else {
        Serial.println("  GPS: ❌ Invalid");
    }

    Serial.printf("  CMD: ");
    switch (currentNavCommand) {
        case NAV_WAIT: Serial.println("WAIT"); break;
        case NAV_TURN_LEFT: Serial.println("LEFT"); break;
        case NAV_TURN_RIGHT: Serial.println("RIGHT"); break;
        case NAV_FORWARD: Serial.println("FORWARD"); break;
        case NAV_STRAFE_LEFT: Serial.println("S_LEFT"); break;
        case NAV_STRAFE_RIGHT: Serial.println("S_RIGHT"); break;
        default: Serial.println("UNKNOWN"); break;
    }
    Serial.println("--------------------------");
}

void OnDataRecv(const esp_now_recv_info_t *mac, const uint8_t *incomingData, int len) {
    if (len == sizeof(CommandMessage)) {
        memcpy(&receivedCmd, incomingData, sizeof(receivedCmd));
        Serial.printf("Received Cmd: %s\n", receivedCmd.command);
         // --- NEW COMMAND CHECK ---
        if (strcmp(receivedCmd.command, "INIT") == 0) {
            triggerInitRoutine = true; // Trigger the flag
            Serial.println(">>> INIT REQUEST RECEIVED. Queuing routine...");
        }
        // -------------------------
        
        else if (strcmp(receivedCmd.command, "POST") == 0) {
            bool isAtHBuilding = (sharedData.state == STATE_REACHED); // Check if mission completed

            if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
                // --- Return to Start Logic with Nearest Waypoint ---
                if (isAtHBuilding) {
                     // 1. Convert current GPS to Local to find distance
                     float cN, cE;
                     geo.toLocalCoordinates(sharedData.lat, sharedData.lon, cN, cE);
                     
                     // 2. Find Nearest Waypoint Index
                     int closestIdx = TOTAL_WAYPOINTS - 1; 
                     float minDistance = 100000.0f;
                     
                     for (int i = 0; i < TOTAL_WAYPOINTS; i++) {
                         float d = geo.getLocalDistance(cN, cE, localPath[i].n, localPath[i].e);
                         if (d < minDistance) {
                             minDistance = d;
                             closestIdx = i;
                         }
                     }
                     
                     // 3. Set up Return Mission
                     sharedData.current_wp_index = closestIdx;
                     currentMissionStep = STEP_RETURN_NAV; 
                     currentMissionType = MISSION_RETURN;
                     sharedData.is_mission_active = true;
                     sharedData.state = STATE_WALKING;
                     
                     Serial.printf(">>> START RETURN MISSION. Found closest WP: %d (Dist: %.2fm)\n", closestIdx, minDistance);
                } else {
                     // Regular POST Mission (Start -> Pt1 -> Blind)
                     sharedData.is_mission_active = true;
                     sharedData.current_wp_index = 0;
                     currentMissionStep = STEP_NAV_TO_POINT1; 
                     sharedData.state = STATE_WALKING;
                     currentMissionType = MISSION_POST_ONLY;
                     Serial.println(">>> START POST MISSION: Point 1 -> Blind 1 Align -> PAUSE");
                }
                xSemaphoreGive(dataMutex);
            }
        }
        else if (strcmp(receivedCmd.command, "H_BUILDING") == 0) {
            if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
                if (currentMissionStep == STEP_BM1_PAUSED) {
                    currentMissionStep = STEP_BM1_FWD_TO_PT2;
                    Serial.println(">>> RESUMING MISSION TO H_BUILDING");
                } 
                else if (sharedData.is_mission_active && currentMissionType == MISSION_POST_ONLY) {
                    currentMissionType = MISSION_FULL;
                    Serial.println(">>> UPGRADED MISSION TO FULL (H_BUILDING)");
                }
                else {
                    sharedData.current_wp_index = 0;
                    currentMissionStep = STEP_NAV_TO_POINT1; 
                    Serial.println(">>> START FULL H_BUILDING MISSION");
                }

                sharedData.is_mission_active = true;
                sharedData.state = STATE_WALKING;
                currentMissionType = MISSION_FULL;
                xSemaphoreGive(dataMutex);
            }
        }
        else if (strcmp(receivedCmd.command, "STOP") == 0) {
            // Toggle the pause state
            isMissionPaused = !isMissionPaused; 

            if (isMissionPaused) {
                // === PAUSE (BRAKES ON) ===
                emergencyStopTriggered = true; 
                Serial.println(">>> PAUSED (Keeping Mission State)");
            }  
            else {
                // === RESUME (BRAKES OFF) ===
                emergencyStopTriggered = false; // Unlatch
                Serial.println(">>> RESUMING Mission from last point...");
            }
        }
    }
}
void OnDataSent(const wifi_tx_info_t *mac, esp_now_send_status_t status) {}

void convertPathToLocal() {
    geo.setOrigin(WayPoints::MONUMENT_LAT, WayPoints::MONUMENT_LON);
    for (int i = 0; i < TOTAL_WAYPOINTS; i++) {
        geo.toLocalCoordinates(gpsPath[i].lat, gpsPath[i].lon, localPath[i].n, localPath[i].e);
    }
}

void prepareLidarScan() {
    if (isSendingLidar) return;
    totalLidarPoints = lidar.getPointCount();
    if (totalLidarPoints > 0) {
        totalLidarPackets = (totalLidarPoints + LIDAR_POINTS_PER_PACKET - 1) / LIDAR_POINTS_PER_PACKET;
        currentLidarPacketIndex = 0;
        isSendingLidar = true;
    }
}

void sendNextLidarPacket() {
    if (!isSendingLidar) return;
    if (currentLidarPacketIndex >= totalLidarPackets) { isSendingLidar = false; return; }
    const LidarPoint *points = lidar.getPoints();
    LidarDataPacket packet;
    packet.total_packets = totalLidarPackets;
    packet.packet_index = currentLidarPacketIndex;
    int start = currentLidarPacketIndex * LIDAR_POINTS_PER_PACKET;
    int toCopy = min((int)LIDAR_POINTS_PER_PACKET, (int)(totalLidarPoints - start));
    if (toCopy > 0) {
        memcpy(packet.points, &points[start], toCopy * sizeof(LidarPoint));
        esp_now_send(espCamAddress, (uint8_t *)&packet, sizeof(packet));
    }
    currentLidarPacketIndex++;
} 