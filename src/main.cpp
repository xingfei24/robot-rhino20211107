#include <Arduino.h>
#include "QuadrupedRobot.h"

QuadrupedRobot robot;

// Gait parameters
const float TROT_CYCLE_TIME = 800.0f;    // ms
const float WALK_CYCLE_TIME = 1200.0f;   // ms
const float GALLOP_CYCLE_TIME = 600.0f;  // ms
const float BOUND_CYCLE_TIME = 500.0f;   // ms

// Gait execution functions with mirroring support
void executeTrot(int cycles = 1, float velocity = 0.5f) {
    robot.setGait(TROT);
    robot.setMaxSpeed(velocity);
    robot.setStepHeight(30.0f);
    robot.setCycleTime(TROT_CYCLE_TIME);
    
    // Diagonal pairs move together
    float phaseOffsets[] = {0.0f, 0.5f, 0.5f, 0.0f};
    for (int i = 0; i < 4; i++) {
        robot.setLegPhaseOffset(i, phaseOffsets[i]);
    }
    
    robot.walk(cycles, velocity);
}

void executeWalk(int cycles = 1, float velocity = 0.4f) {
    robot.setGait(WALK);
    robot.setMaxSpeed(velocity);
    robot.setStepHeight(35.0f);
    robot.setCycleTime(WALK_CYCLE_TIME);
    
    // Sequential leg movement with proper left-right coordination
    float phaseOffsets[] = {0.0f, 0.25f, 0.75f, 0.5f};
    for (int i = 0; i < 4; i++) {
        robot.setLegPhaseOffset(i, phaseOffsets[i]);
    }
    
    robot.walk(cycles, velocity);
}

void executeGallop(int cycles = 1, float velocity = 0.7f) {
    robot.setGait(GALLOP);
    robot.setMaxSpeed(velocity);
    robot.setStepHeight(45.0f);
    robot.setCycleTime(GALLOP_CYCLE_TIME);
    
    // Front legs move slightly apart, followed by back legs
    float phaseOffsets[] = {0.0f, 0.1f, 0.6f, 0.5f};
    for (int i = 0; i < 4; i++) {
        robot.setLegPhaseOffset(i, phaseOffsets[i]);
    }
    
    robot.walk(cycles, velocity);
}

void executeBound(int cycles = 1, float velocity = 0.6f) {
    robot.setGait(BOUND);
    robot.setMaxSpeed(velocity);
    robot.setStepHeight(40.0f);
    robot.setCycleTime(BOUND_CYCLE_TIME);
    
    // Front pair and back pair move together
    float phaseOffsets[] = {0.0f, 0.0f, 0.5f, 0.5f};
    for (int i = 0; i < 4; i++) {
        robot.setLegPhaseOffset(i, phaseOffsets[i]);
    }
    
    robot.walk(cycles, velocity);
}

void executeTurn(float angle, Direction direction) {
    robot.setGait(WALK);
    robot.setMaxSpeed(0.3f);  // Slower speed for turning
    robot.setStepHeight(25.0f);
    
    if (direction == LEFT) {
        float phaseOffsets[] = {0.5f, 0.0f, 0.0f, 0.5f};
        for (int i = 0; i < 4; i++) {
            robot.setLegPhaseOffset(i, phaseOffsets[i]);
        }
    } else {  // RIGHT
        float phaseOffsets[] = {0.0f, 0.5f, 0.5f, 0.0f};
        for (int i = 0; i < 4; i++) {
            robot.setLegPhaseOffset(i, phaseOffsets[i]);
        }
    }
    
    robot.turn(angle);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing quadruped robot...");
    
    robot.init();
    robot.setMaxSpeed(0.5f);
    robot.setStepHeight(30.0f);
    robot.setBodyHeight(100.0f);
    robot.setPIDGains(0.8f, 0.2f, 0.1f);
    
    robot.calibrateSensors();
    delay(1000);
    
    Serial.println("Robot initialized and ready.");
}

void loop() {
    static int currentGait = 0;
    static unsigned long lastGaitChange = 0;
    static const unsigned long GAIT_DURATION = 5000; // 5 seconds per gait
    static const int DEMO_SEQUENCE_LENGTH = 5;
    
    unsigned long currentTime = millis();
    
    // Switch gaits every GAIT_DURATION milliseconds
    if (currentTime - lastGaitChange >= GAIT_DURATION) {
        robot.stand(); // Reset to standing position between gaits
        delay(500);   // Short pause between gaits
        
        switch (currentGait) {
            case 0:
                Serial.println("Executing Walk");
                executeWalk(2);
                break;
            case 1:
                Serial.println("Executing Trot");
                executeTrot(3);
                break;
            case 2:
                Serial.println("Executing Turn Left");
                executeTurn(90.0f, LEFT);
                break;
            case 3:
                Serial.println("Executing Gallop");
                executeGallop(3);
                break;
            case 4:
                Serial.println("Executing Bound");
                executeBound(3);
                break;
        }
        
        currentGait = (currentGait + 1) % DEMO_SEQUENCE_LENGTH;
        lastGaitChange = currentTime;
    }
    
    // Main control loop
    robot.update();
    
    // Safety checks and monitoring
    robot.monitorHealth();
    
    // Small delay to prevent CPU overload
    delay(10);
}