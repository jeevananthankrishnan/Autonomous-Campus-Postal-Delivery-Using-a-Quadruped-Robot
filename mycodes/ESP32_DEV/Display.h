#pragma once
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

enum SystemState {
    STATE_INITIALIZING,
    STATE_WAITING_FOR_GPS,
    STATE_WALKING,
    STATE_READY,
    STATE_COMMAND_FEEDBACK,
    STATE_REACHED
};

// === UPDATED ENUM ===
enum NavCommand {
    NAV_WAIT,
    NAV_TURN_LEFT,
    NAV_TURN_RIGHT,
    NAV_FORWARD,
    NOT_CLOSE,
    NAV_STOPPED,       // <--- For "W_ORD"
    NAV_STRAFE_LEFT,   // <--- REQUIRED by ESPMain.ino
    NAV_STRAFE_RIGHT   // <--- REQUIRED by ESPMain.ino
};

class Display {
public:
    Display() : m_display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {}

    bool begin() {
        if (!m_display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
            return false;
        }
        m_display.clearDisplay();
        m_display.setTextColor(SSD1306_WHITE);
        m_display.display();
        return true;
    }

    void update(SystemState state, NavCommand navCmd, uint16_t frontDistance, 
                uint16_t leftDistance, uint16_t rightDistance, 
                float yaw, float monumentDistance) {
        
        m_display.clearDisplay();
        m_display.setTextSize(2);
        m_display.setCursor(0, 0);

        switch (state) {
            case STATE_INITIALIZING:
                m_display.println("TEAM 08");
                m_display.setCursor(0, 20);
                m_display.println("STARTING");
                break;

            case STATE_WAITING_FOR_GPS:
                m_display.println("GPS WAIT");
                m_display.setCursor(0, 20);
                m_display.printf("Yaw:%.0fd", yaw);
                m_display.setCursor(0, 40);
                printNavCommand(navCmd);
                break;

            case STATE_WALKING:
            case STATE_READY:
                // 1. Check for Front Obstacle First (< 50cm)
                if (frontDistance > 0 && frontDistance < 50) {
                    m_display.println("OBSTACLE"); 
                    m_display.setCursor(0, 20);
                    m_display.printf("Fr:%dcm", frontDistance);
                    m_display.setCursor(0, 40);
                    
                    if (leftDistance > rightDistance) {
                        m_display.println("S_LEFT");
                    } else {
                        m_display.println("S_RIGHT");
                    }
                } 
                // 2. Normal Navigation or Side Avoidance
                else {
                    printNavCommand(navCmd); // Prints D_ORD, W_ORD, or S_LEFT/RIGHT
                    
                    m_display.setCursor(0, 20);
                    m_display.printf("Fr:%dcm", frontDistance);
                    
                    m_display.setCursor(0, 40);
                    if (state == STATE_READY) {
                        m_display.println("READY");
                    } else {
                        m_display.printf("Dist:%.0fm", monumentDistance);
                    }
                }
                break;

            case STATE_COMMAND_FEEDBACK:
                m_display.setTextSize(3);
                m_display.println("ERROR");
                m_display.setTextSize(2);
                m_display.setCursor(0, 32);
                m_display.println("STATE");
                break;

            case STATE_REACHED:
                m_display.setTextSize(3); 
                m_display.setCursor(0, 20); 
                m_display.println("REACHED");
                break;
        }

        m_display.display();
    }

private:
    Adafruit_SSD1306 m_display;

    void printNavCommand(NavCommand cmd) {
        m_display.print("CMD:");
        switch (cmd) {
            case NAV_WAIT:         m_display.println("D_ORD"); break;  // Logic Pause
            case NAV_STOPPED:      m_display.println("W_ORD"); break;  // Manual Stop
            case NAV_TURN_LEFT:    m_display.println("R_LEFT"); break; 
            case NAV_TURN_RIGHT:   m_display.println("R_RIGHT"); break;
            case NAV_FORWARD:      m_display.println("FRWD"); break;
            case NOT_CLOSE:        m_display.println("FAR"); break;
            case NAV_STRAFE_LEFT:  m_display.println("S_LEFT"); break;
            case NAV_STRAFE_RIGHT: m_display.println("S_RIGHT"); break;
        }
    }
};