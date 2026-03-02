#pragma once
#include <Arduino.h>
#include "Display.h" 
#include "Lidar.h"

class Navigation
{
public:
    Navigation() {}

    // ==================================================================================
    // 1. LOCAL CARTESIAN PATH (Meters)
    // Used for: STEP_NAV_TO_POINT1 and STEP_RESUME_GPS
    // ==================================================================================
    NavCommand computeLocalPathCommand(float currN, float currE, 
                                       float targetN, float targetE, 
                                       float currentYaw)
    {
        float dN = targetN - currN;
        float dE = targetE - currE;
        
        // Calculate target heading relative to North (0 deg)
        // atan2(y, x) -> atan2(East, North) gives 0 at North, 90 at East
        float targetHeading = atan2(dE, dN) * RAD_TO_DEG;
        if (targetHeading < 0) targetHeading += 360.0f;

        float error = targetHeading - currentYaw;
        
        // Normalize error to -180 to +180
        while (error < -180.0f) error += 360.0f;
        while (error > 180.0f)  error -= 360.0f;

        // Tolerance: +/- 15 degrees
        if (abs(error) <= 15.0f) return NAV_FORWARD;
        if (error > 0) return NAV_TURN_RIGHT;
        return NAV_TURN_LEFT;
    }

    // ==================================================================================
    // 2. HEADING HOLD (Compass Drive)
    // Used for: STEP_ALIGN_NORTH (allowForward=false) and STEP_FWD_SEARCH (allowForward=true)
    // ==================================================================================
    NavCommand computeHeadingCommand(float targetHeading, float currentYaw, bool allowForward)
    {
        float error = targetHeading - currentYaw;
        
        // Normalize error
        while (error < -180.0f) error += 360.0f;
        while (error > 180.0f)  error -= 360.0f;

        // Tighter tolerance for specific mission turns (10 degrees)
        // This ensures the robot faces the right way before starting to walk blindly
        if (abs(error) <= 10.0f) {
            if (allowForward) return NAV_FORWARD;
            return NAV_WAIT; // Aligned, but told to wait (just turning in place)
        }

        if (error > 0) return NAV_TURN_RIGHT;
        return NAV_TURN_LEFT;
    }

    // ==================================================================================
    // 3. GPS PATH (Lat/Lon) 
    // Legacy support for direct coordinate navigation
    // ==================================================================================
    NavCommand computePathCommand(double currentLat, double currentLon,
                                  double targetLat, double targetLon,
                                  float currentYaw)
    {
        float targetHeading = calculateBearing(currentLat, currentLon, targetLat, targetLon);
        float error = targetHeading - currentYaw;
        
        while (error < -180.0f) error += 360.0f;
        while (error > 180.0f)  error -= 360.0f;

        if (abs(error) <= 15.0f) return NAV_FORWARD;
        if (error > 0) return NAV_TURN_RIGHT;
        return NAV_TURN_LEFT;
    }

    // ==================================================================================
    // 4. OBSTACLE AVOIDANCE (Sensors)
    // Logic to avoid walls based on US and Lidar
    // ==================================================================================
    NavCommand computeCommand(uint16_t frontUS, uint16_t leftUS, uint16_t rightUS,
                              const LidarPoint* lidarPoints, bool targetNear)
    {
        const uint16_t US_FRONT_CLEAR = 90;  
        const uint16_t US_SIDE_BLOCK = 40;   
        const uint16_t LIDAR_BLOCK_MM = 700; 
        const int LIDAR_WINDOW = 8;          

        // Pass the array pointer to the helper function
        uint16_t frontLidar = lidarAverage(lidarPoints, 0, LIDAR_WINDOW);
        uint16_t leftLidar = lidarAverage(lidarPoints, 90, LIDAR_WINDOW);
        uint16_t rightLidar = lidarAverage(lidarPoints, 270, LIDAR_WINDOW);

        // Case: US Front reads 0 (often Error or Out of Range)
        if (frontUS == 0) {
            if ((leftUS > 0 && leftUS <= US_SIDE_BLOCK)) return NAV_TURN_RIGHT;
            if ((rightUS > 0 && rightUS <= US_SIDE_BLOCK)) return NAV_TURN_LEFT;
            
            bool lidarFrontBlocked = (frontLidar < LIDAR_BLOCK_MM);
            if (lidarFrontBlocked) {
                // If blocked by Lidar, turn towards the more open side
                if (leftLidar > rightLidar + 200) return NAV_TURN_LEFT;
                return NAV_TURN_RIGHT;
            }
            return NAV_FORWARD;
        }

        // Case: Front is clear
        if (frontUS > US_FRONT_CLEAR && frontLidar > LIDAR_BLOCK_MM) {
            return NAV_FORWARD;
        }

        // Case: Front blocked, decide turn based on weighted score
        int leftScore = (leftUS > 0 ? leftUS : 0) + (leftLidar / 10);
        int rightScore = (rightUS > 0 ? rightUS : 0) + (rightLidar / 10);
        return (leftScore > rightScore) ? NAV_TURN_LEFT : NAV_TURN_RIGHT;
    }

private:
    // Helper now takes a direct pointer to the points array
    uint16_t lidarAverage(const LidarPoint* pts, int centerAngle, int window) {
        long sum = 0;
        int count = 0;
        int start = centerAngle - window;
        int end = centerAngle + window;
        for (int a = start; a <= end; a++) {
            int ang = (a + 360) % 360;
            // Access the array directly
            uint16_t d = pts[ang].distance;
            // Ignore invalid (0) or infinity (>8000) readings
            if (d > 0 && d < 8000) { sum += d; count++; }
        }
        return (count == 0) ? 9999 : (sum / count);
    }

    float calculateBearing(double lat1, double lon1, double lat2, double lon2) {
        float dLon = (lon2 - lon1) * DEG_TO_RAD;
        float lat1Rad = lat1 * DEG_TO_RAD;
        float lat2Rad = lat2 * DEG_TO_RAD;
        
        float y = sin(dLon) * cos(lat2Rad);
        float x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(dLon);
        
        float brng = atan2(y, x) * RAD_TO_DEG;
        if (brng < 0) brng += 360.0f;
        return brng;
    }
};