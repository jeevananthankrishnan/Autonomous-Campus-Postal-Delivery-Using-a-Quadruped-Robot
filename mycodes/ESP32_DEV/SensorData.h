#pragma once
#include <Arduino.h>

// ESP-NOW message structure for basic sensor data
typedef struct {
    uint16_t us_front, us_back, us_left, us_right, us_incline;
    float imu_roll, imu_pitch, imu_yaw;
    double gps_lat, gps_lon;
    bool gps_valid;
    uint16_t lidar_closest_dist;
    float lidar_closest_angle;
    bool lidar_valid;
    uint32_t timestamp;
    float local_N_to_monument;
    float local_E_to_monument;
    float gps_distance_to_monument;
    
    // --- NEW ADDITION FOR DASHBOARD DIRECTIONS ---
    int8_t nav_command; // 0=Wait, 1=Left, 2=Right, 3=Forward, 4=NotClose
} struct_message;

// Command message structure
typedef struct {
    char command[16];
} CommandMessage;

// Lidar packet structure
#define LIDAR_POINTS_PER_PACKET 38
typedef struct {
    uint8_t packet_index;
    uint8_t total_packets;
    LidarPoint points[LIDAR_POINTS_PER_PACKET];
} LidarDataPacket;

// Utility function
inline uint16_t normalizeUS(uint16_t rawValue) {
    return (rawValue == 0) ? 999 : rawValue;
}