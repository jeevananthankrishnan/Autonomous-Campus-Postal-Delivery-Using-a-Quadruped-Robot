#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ====================== LD20 PACKET CONSTANTS ======================
#define LD20_HEADER      0x54
#define LD20_VERLEN      0x2C
#define LD20_PACKET_SIZE 47

static const uint16_t DETECTION_RANGE_MM = 4000;
static const uint8_t  ANGLE_BIN_SIZE_DEG = 1;
static const uint16_t NUM_BINS           = 360;

struct LidarPoint {
  float    angle;       // Angle in degrees
  uint16_t distance;    // Distance in mm
  uint8_t  intensity;   
};

class LidarLD20 {
public:
  LidarLD20(HardwareSerial &serialPort, uint32_t baud = 230400)
  : m_serial(serialPort),
    m_baud(baud),
    m_bufIndex(0),
    m_speed(0)
  {
    // Initialize Mutex for thread safety
    m_mutex = xSemaphoreCreateMutex();
    clearPoints();
  }

  void begin(uint8_t rxPin, uint8_t txPin) {
    m_serial.begin(m_baud, SERIAL_8N1, rxPin, txPin);
  }

  // Reads as many bytes as available. Returns true if a new packet was parsed.
  bool update() {
    bool anyPacketParsed = false;
    // Limit loop to avoid blocking main loop too long, but read enough to empty buffer
    int maxBytes = 128; 
    while (m_serial.available() && maxBytes-- > 0) {
      uint8_t byte = m_serial.read();

      // Sync header
      if (m_bufIndex == 0 && byte != LD20_HEADER) {
        continue;
      }

      m_buffer[m_bufIndex++] = byte;

      if (m_bufIndex >= LD20_PACKET_SIZE) {
        if (validatePacket()) {
          parsePacket();
          anyPacketParsed = true;
        }
        m_bufIndex = 0; // Reset for next packet
      }
    }
    return anyPacketParsed;
  }

  // Direct access to the live array (Used by ESP-NOW sender)
  // Note: This is still technically unsafe during a write, but acceptable for just visualization
  const LidarPoint* getPoints() const {
    return m_points;
  }

  // NEW: Thread-Safe Copy Function for Logic Task (Core 0)
  void getSafePoints(LidarPoint* destination) {
    if (xSemaphoreTake(m_mutex, (TickType_t)5) == pdTRUE) {
      memcpy(destination, m_points, sizeof(LidarPoint) * NUM_BINS);
      xSemaphoreGive(m_mutex);
    }
  }

  uint16_t getPointCount() const {
    return NUM_BINS;
  }

  // Standard finding logic - Now Protected by Mutex
  int findClosestObstacle(uint16_t &minDistance) {
    minDistance = 65535;
    int minIndex = -1;
    
    // Lock before reading
    if (xSemaphoreTake(m_mutex, (TickType_t)5) == pdTRUE) {
      for (uint16_t i = 0; i < NUM_BINS; i++) {
        if (m_points[i].distance > 0 && m_points[i].distance < minDistance) {
          minDistance = m_points[i].distance;
          minIndex    = (int)i;
        }
      }
      xSemaphoreGive(m_mutex);
    }
    return minIndex;
  }

private:
  HardwareSerial &m_serial;
  uint32_t m_baud;
  uint8_t m_buffer[LD20_PACKET_SIZE];
  int     m_bufIndex;
  float   m_speed;
  
  // LIVE points array (updated immediately)
  LidarPoint m_points[NUM_BINS];
  
  // Mutex Handle
  SemaphoreHandle_t m_mutex;

  bool validatePacket() {
    if (m_buffer[0] != LD20_HEADER || m_buffer[1] != LD20_VERLEN) return false;
    uint8_t checksum = 0;
    for (int i = 0; i < LD20_PACKET_SIZE - 1; i++) {
      checksum = (uint8_t)(checksum + m_buffer[i]); // Explicit cast for clarity
    }
    return (checksum == m_buffer[LD20_PACKET_SIZE - 1]);
  }

  void clearPoints() {
    for (uint16_t i = 0; i < NUM_BINS; i++) {
      m_points[i].angle     = (float)i;
      m_points[i].distance  = 0;
      m_points[i].intensity = 0;
    }
  }

  void parsePacket() {
    // Speed: 2 bytes at offset 2 (deg/sec)
    m_speed = (float)((m_buffer[3] << 8) | m_buffer[2]) / 100.0f;

    // Start Angle: 2 bytes at offset 4
    float startAngle = (float)((m_buffer[5] << 8) | m_buffer[4]) / 100.0f;

    // LD20 packet contains 12 points
    // Step angle is approx speed/sample_rate, but usually ~0.8-0.9 degrees per point
    // We can assume strict linear interpolation for the 12 points.
    float step = 0.9f; // Approximate, or calculated based on speed

    // Lock before writing to m_points
    if (xSemaphoreTake(m_mutex, (TickType_t)2) == pdTRUE) {
      for (int i = 0; i < 12; i++) {
        int offset = 6 + (i * 3);
        uint16_t dist = (uint16_t)((m_buffer[offset+1] << 8) | m_buffer[offset]);
        // uint8_t conf = m_buffer[offset+2]; // Intensity, unused

        // Calculate exact angle for this point
        float currentAngle = startAngle + (i * step);
        if (currentAngle >= 360.0f) currentAngle -= 360.0f;

        // Filter invalid or out of range
        if (dist > DETECTION_RANGE_MM) dist = 0;

        // Map to 0-359 Integer Index
        int binIndex = (int)(currentAngle + 0.5f); // Round to nearest degree
        if (binIndex >= 360) binIndex = 0;
        
        // === DIRECT UPDATE (No Buffering) ===
        m_points[binIndex].angle = currentAngle;
        m_points[binIndex].distance = dist;
      }
      xSemaphoreGive(m_mutex);
    }
  }
};