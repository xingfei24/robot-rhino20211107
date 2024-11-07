#include "QuadrupedRobot.h"
#include <math.h>

QuadrupedRobot::QuadrupedRobot() : 
    pwm(), mpu(), moving(false), currentGait(TROT), maxSpeed(1.0f), bodyHeight(100.0f),
    state(LEARNING) {
    
    nn = new NeuralNetwork(14, 20, 8);  // 14 inputs, 20 hidden, 8 outputs
    
    gaitParams = {
        .stepHeight = 30.0f,
        .strideLength = 50.0f,
        .cycleTime = 1000.0f,
        .phaseOffsets = {0.0f, 0.25f, 0.5f, 0.75f}
    };
    
    initServoConfigs();
    initLegConfigs();
}

QuadrupedRobot::~QuadrupedRobot() {
    delete nn;
}

void QuadrupedRobot::init() {
    Wire.begin();
    pwm.begin();
    pwm.setPWMFreq(50);  // Standard servo frequency
    
    if (!mpu.begin()) {
        Serial.println("MPU6050 initialization failed!");
        return;
    }
    
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    calibrateSensors();
    stand();
}

void QuadrupedRobot::initServoConfigs() {
    // Configure each servo with mirroring support
    for (int i = 0; i < TOTAL_SERVOS; i++) {
        servoConfigs[i] = {
            .pin = i,
            .minPulse = 150,
            .maxPulse = 600,
            .minAngle = -90.0f,
            .maxAngle = 90.0f,
            .offset = 0.0f,
            .reversed = false,
            .isMirrored = (i % 2 == 1)  // Mirror every second servo
        };
    }
}

void QuadrupedRobot::initLegConfigs() {
    // Initialize leg configurations with proper mirroring
    const float X_OFFSET = 100.0f;
    const float Y_OFFSET = 100.0f;
    
    legConfigs[FRONT_RIGHT] = {X_OFFSET, Y_OFFSET, -bodyHeight, false, 0.0f};
    legConfigs[FRONT_LEFT] = {X_OFFSET, -Y_OFFSET, -bodyHeight, true, 0.0f};
    legConfigs[BACK_RIGHT] = {-X_OFFSET, Y_OFFSET, -bodyHeight, false, 0.0f};
    legConfigs[BACK_LEFT] = {-X_OFFSET, -Y_OFFSET, -bodyHeight, true, 0.0f};
    
    for (int i = 0; i < NUM_LEGS; i++) {
        isGrounded[i] = true;
        lastContactTime[i] = millis();
    }
}

void QuadrupedRobot::update() {
    updateMpuData();
    
    // Basic balance and movement updates
    if (moving) {
        balance();
    }
    
    validateMovement();
    optimizeEnergyUsage();
}

void QuadrupedRobot::calibrateSensors() {
    Serial.println("Calibrating sensors...");
    float accelBias[3] = {0, 0, 0};
    float gyroBias[3] = {0, 0, 0};
    const int samples = 100;
    
    for (int i = 0; i < samples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        accelBias[0] += a.acceleration.x;
        accelBias[1] += a.acceleration.y;
        accelBias[2] += a.acceleration.z - 9.81; // Remove gravity
        
        gyroBias[0] += g.gyro.x;
        gyroBias[1] += g.gyro.y;
        gyroBias[2] += g.gyro.z;
        
        delay(10);
    }
    
    // Calculate average biases
    for (int i = 0; i < 3; i++) {
        accelBias[i] /= samples;
        gyroBias[i] /= samples;
    }
    
    Serial.println("Calibration complete");
}

void QuadrupedRobot::setPIDGains(float kp, float ki, float kd) {
    pidGains[0] = kp;
    pidGains[1] = ki;
    pidGains[2] = kd;
}

void QuadrupedRobot::setMaxSpeed(float speed) {
    maxSpeed = constrain(speed, 0.0f, 1.0f);
}

void QuadrupedRobot::setStepHeight(float height) {
    gaitParams.stepHeight = height;
}

void QuadrupedRobot::setBodyHeight(float height) {
    bodyHeight = height;
    stand(); // Update standing position with new height
}

void QuadrupedRobot::setCycleTime(float time) {
    gaitParams.cycleTime = time;
}

void QuadrupedRobot::setLegPhaseOffset(int legIndex, float offset) {
    if (legIndex >= 0 && legIndex < NUM_LEGS) {
        gaitParams.phaseOffsets[legIndex] = offset;
    }
}

void QuadrupedRobot::walk(int cycles, float velocity, Direction direction) {
    if (velocity > maxSpeed) velocity = maxSpeed;
    moving = true;
    
    unsigned long startTime = millis();
    int currentCycle = 0;
    
    while (moving && currentCycle < cycles) {
        unsigned long currentTime = millis();
        float phase = ((currentTime - startTime) % (unsigned long)gaitParams.cycleTime) / gaitParams.cycleTime;
        
        generateWalkingGait(phase, velocity, direction);
        
        float pitchCorrection, rollCorrection;
        calculateBalanceCorrection(pitchCorrection, rollCorrection);
        applyBalanceCorrection(pitchCorrection, rollCorrection);
        
        if (!isBalanced()) {
            stabilize();
            break;
        }
        
        if (currentTime - startTime >= (currentCycle + 1) * gaitParams.cycleTime) {
            currentCycle++;
        }
        
        delay(10);
    }
    
    moving = false;
}

void QuadrupedRobot::stand() {
    moving = false;
    
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        float hipAngle, kneeAngle;
        calculateInverseKinematics(
            legConfigs[leg].x,
            legConfigs[leg].y,
            -bodyHeight,
            hipAngle,
            kneeAngle,
            legConfigs[leg].isMirrored
        );
        
        setServoAngle(leg * 2, hipAngle);
        setServoAngle(leg * 2 + 1, kneeAngle);
        isGrounded[leg] = true;
    }
}

void QuadrupedRobot::balance() {
    float pitchCorrection, rollCorrection;
    calculateBalanceCorrection(pitchCorrection, rollCorrection);
    applyBalanceCorrection(pitchCorrection, rollCorrection);
}

void QuadrupedRobot::stabilize() {
    updateMpuData();
    
    if (abs(mpuData.pitch) > 45.0f || abs(mpuData.roll) > 45.0f) {
        emergencyStop();
        return;
    }
    
    balance();
    
    if (isBalanced()) {
        stand();
    }
}

void QuadrupedRobot::setGait(GaitType gait) {
    currentGait = gait;
    
    switch (gait) {
        case TROT:
            gaitParams.phaseOffsets[0] = 0.0f;
            gaitParams.phaseOffsets[1] = 0.5f;
            gaitParams.phaseOffsets[2] = 0.5f;
            gaitParams.phaseOffsets[3] = 0.0f;
            break;
        case WALK:
            gaitParams.phaseOffsets[0] = 0.0f;
            gaitParams.phaseOffsets[1] = 0.25f;
            gaitParams.phaseOffsets[2] = 0.5f;
            gaitParams.phaseOffsets[3] = 0.75f;
            break;
        case GALLOP:
            gaitParams.phaseOffsets[0] = 0.0f;
            gaitParams.phaseOffsets[1] = 0.1f;
            gaitParams.phaseOffsets[2] = 0.5f;
            gaitParams.phaseOffsets[3] = 0.6f;
            break;
        case BOUND:
            gaitParams.phaseOffsets[0] = 0.0f;
            gaitParams.phaseOffsets[1] = 0.0f;
            gaitParams.phaseOffsets[2] = 0.5f;
            gaitParams.phaseOffsets[3] = 0.5f;
            break;
    }
}

void QuadrupedRobot::jump(float height, float duration) {
    // Prepare for jump
    crouch(bodyHeight * 0.7f);
    delay(200);
    
    // Jump motion
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        float hipAngle, kneeAngle;
        calculateInverseKinematics(
            legConfigs[leg].x,
            legConfigs[leg].y,
            -bodyHeight + height,
            hipAngle,
            kneeAngle,
            legConfigs[leg].isMirrored
        );
        
        setServoAngle(leg * 2, hipAngle);
        setServoAngle(leg * 2 + 1, kneeAngle);
    }
    
    delay(duration);
    
    // Land
    stand();
}

void QuadrupedRobot::turn(float angle) {
    const float turnRadius = sqrt(
        legConfigs[0].x * legConfigs[0].x + 
        legConfigs[0].y * legConfigs[0].y
    );
    
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        float currentAngle = atan2(legConfigs[leg].y, legConfigs[leg].x);
        float newAngle = currentAngle + angle * M_PI / 180.0f;
        
        float hipAngle, kneeAngle;
        calculateInverseKinematics(
            cos(newAngle) * turnRadius,
            sin(newAngle) * turnRadius,
            -bodyHeight,
            hipAngle,
            kneeAngle,
            legConfigs[leg].isMirrored
        );
        
        setServoAngle(leg * 2, hipAngle);
        setServoAngle(leg * 2 + 1, kneeAngle);
    }
}

void QuadrupedRobot::crouch(float height) {
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        float hipAngle, kneeAngle;
        calculateInverseKinematics(
            legConfigs[leg].x,
            legConfigs[leg].y,
            -height,
            hipAngle,
            kneeAngle,
            legConfigs[leg].isMirrored
        );
        
        setServoAngle(leg * 2, hipAngle);
        setServoAngle(leg * 2 + 1, kneeAngle);
    }
}

void QuadrupedRobot::stretch() {
    // Stretch sequence
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        // Lift leg
        float hipAngle, kneeAngle;
        calculateInverseKinematics(
            legConfigs[leg].x,
            legConfigs[leg].y,
            -bodyHeight + 50.0f,
            hipAngle,
            kneeAngle,
            legConfigs[leg].isMirrored
        );
        
        setServoAngle(leg * 2, hipAngle);
        setServoAngle(leg * 2 + 1, kneeAngle);
        
        delay(500);
        
        // Return to standing
        calculateInverseKinematics(
            legConfigs[leg].x,
            legConfigs[leg].y,
            -bodyHeight,
            hipAngle,
            kneeAngle,
            legConfigs[leg].isMirrored
        );
        
        setServoAngle(leg * 2, hipAngle);
        setServoAngle(leg * 2 + 1, kneeAngle);
        
        delay(500);
    }
}

bool QuadrupedRobot::isBalanced() const {
    const float threshold = 5.0f; // degrees
    return abs(mpuData.pitch) < threshold && abs(mpuData.roll) < threshold;
}

bool QuadrupedRobot::isMoving() const {
    return moving;
}

float QuadrupedRobot::getBatteryLevel() const {
    // Implement battery level reading
    return 0.0f;  // Placeholder
}

const MpuData& QuadrupedRobot::getSensorData() const {
    return mpuData;
}

void QuadrupedRobot::emergencyStop() {
    moving = false;
    state = EMERGENCY;
    stand();
    Serial.println("Emergency stop triggered!");
}

void QuadrupedRobot::updateMpuData() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    mpuData.accel_x = a.acceleration.x;
    mpuData.accel_y = a.acceleration.y;
    mpuData.accel_z = a.acceleration.z;
    
    mpuData.gyro_x = g.gyro.x;
    mpuData.gyro_y = g.gyro.y;
    mpuData.gyro_z = g.gyro.z;
    
    mpuData.temp = temp.temperature;
    
    mpuData.calculateAngles();
}

void QuadrupedRobot::calculateBalanceCorrection(float& pitchCorrection, float& rollCorrection) {
    const float dt = 0.01f; // Fixed time step for stability
    
    float pitch = mpuData.pitch;
    float roll = mpuData.roll;
    
    // Simple P controller for stability
    pitchCorrection = pidGains[0] * pitch;
    rollCorrection = pidGains[0] * roll;
}

void QuadrupedRobot::applyBalanceCorrection(float pitchCorrection, float rollCorrection) {
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        float z = -bodyHeight;
        
        // Apply pitch correction (front/back)
        if (leg < 2) z += pitchCorrection * 20.0f;
        else z -= pitchCorrection * 20.0f;
        
        // Apply roll correction (left/right)
        if (leg % 2) z += rollCorrection * 20.0f;
        else z -= rollCorrection * 20.0f;
        
        float hipAngle, kneeAngle;
        calculateInverseKinematics(
            legConfigs[leg].x,
            legConfigs[leg].y,
            z,
            hipAngle,
            kneeAngle,
            legConfigs[leg].isMirrored
        );
        
        setServoAngle(leg * 2, hipAngle);
        setServoAngle(leg * 2 + 1, kneeAngle);
    }
}

void QuadrupedRobot::generateWalkingGait(float phase, float velocity, Direction direction) {
    float stride = velocity * gaitParams.strideLength;
    
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        float legPhase = fmod(phase + gaitParams.phaseOffsets[leg], 1.0f);
        float x = legConfigs[leg].x;
        float y = legConfigs[leg].y;
        float z = -bodyHeight;
        
        if (legPhase < 0.5f) {
            // Stance phase
            float stanceX = cos(legPhase * M_PI * 2) * stride;
            float stanceY = 0;
            
            switch (direction) {
                case BACKWARD:
                    stanceX = -stanceX;
                    break;
                case LEFT:
                    stanceY = stanceX;
                    stanceX = 0;
                    break;
                case RIGHT:
                    stanceY = -stanceX;
                    stanceX = 0;
                    break;
                default: // FORWARD
                    break;
            }
            
            x += stanceX;
            y += stanceY;
            isGrounded[leg] = true;
            lastContactTime[leg] = millis();
        } else {
            // Swing phase
            z = -bodyHeight + sin((legPhase - 0.5f) * M_PI * 2) * gaitParams.stepHeight;
            isGrounded[leg] = false;
        }
        
        float hipAngle, kneeAngle;
        calculateInverseKinematics(x, y, z, hipAngle, kneeAngle, legConfigs[leg].isMirrored);
        
        setServoAngle(leg * 2, hipAngle);
        setServoAngle(leg * 2 + 1, kneeAngle);
    }
}

void QuadrupedRobot::calculateLegForces() {
    // Implement force calculation for each leg
    // This is a placeholder for actual force sensor implementation
}

void QuadrupedRobot::calculateInverseKinematics(float x, float y, float z, 
                                               float& hipAngle, float& kneeAngle,
                                               bool isMirrored) {
    const float L1 = 50.0f; // Upper leg length
    const float L2 = 50.0f; // Lower leg length
    
    // Mirror the Y coordinate for left legs
    if (isMirrored) {
        y = -y;
    }
    
    // Calculate leg length and angle in the x-y plane
    float L = sqrt(x*x + y*y);
    float gamma = atan2(y, x);
    
    // Calculate leg length in the x-z plane
    float D = sqrt(L*L + z*z);
    
    // Check if position is reachable
    if (D > L1 + L2) {
        float scale = (L1 + L2) / D;
        x *= scale;
        y *= scale;
        z *= scale;
        D = L1 + L2;
    }
    
    // Calculate joint angles using cosine law
    float alpha = acos((L1*L1 + D*D - L2*L2)/(2*L1*D)) + atan2(z, L);
    float beta = acos((L1*L1 + L2*L2 - D*D)/(2*L1*L2));
    
    // Convert to degrees and apply mirroring if needed
    hipAngle = clampAngle(gamma * 180.0f/M_PI, -90.0f, 90.0f);
    kneeAngle = clampAngle((M_PI - beta) * 180.0f/M_PI, 0.0f, 150.0f);
    
    if (isMirrored) {
        hipAngle = mirrorAngle(hipAngle, true);
    }
}

void QuadrupedRobot::setServoAngle(int servoIndex, float angle) {
    if (servoIndex >= TOTAL_SERVOS) return;
    
    const ServoConfig& config = servoConfigs[servoIndex];
    float adjustedAngle = angle + config.offset;
    
    if (config.reversed) {
        adjustedAngle = -adjustedAngle;
    }
    
    if (config.isMirrored) {
        adjustedAngle = mirrorAngle(adjustedAngle, true);
    }
    
    adjustedAngle = clampAngle(adjustedAngle, config.minAngle, config.maxAngle);
    float pulse = map(adjustedAngle, -90, 90, config.minPulse, config.maxPulse);
    
    pwm.setPWM(config.pin, 0, (int)pulse);
    currentLegAngles[servoIndex] = angle;
}

float QuadrupedRobot::clampAngle(float angle, float min, float max) {
    return fmax(min, fmin(max, angle));
}

float QuadrupedRobot::mirrorAngle(float angle, bool isMirrored) {
    return isMirrored ? -angle : angle;
}

void QuadrupedRobot::validateMovement() {
    for (int i = 0; i < NUM_LEGS; i++) {
        // Check leg extension limits
        float x = legConfigs[i].x;
        float y = legConfigs[i].y;
        float z = legConfigs[i].z;
        
        float extension = sqrt(x*x + y*y + z*z);
        if (extension > 100.0f) { // Maximum leg extension
            emergencyStop();
            Serial.println("Leg extension limit exceeded!");
            return;
        }
        
        // Check servo angle limits
        if (abs(currentLegAngles[i*2]) > 90.0f || 
            currentLegAngles[i*2+1] < 0.0f || 
            currentLegAngles[i*2+1] > 150.0f) {
            emergencyStop();
            Serial.println("Servo angle limit exceeded!");
            return;
        }
    }
}

void QuadrupedRobot::optimizeEnergyUsage() {
    // Implement power optimization strategies
    // This is a placeholder for actual power management
}

void QuadrupedRobot::monitorHealth() {
    // Check orientation limits
    if (abs(mpuData.pitch) > 45.0f || abs(mpuData.roll) > 45.0f) {
        emergencyStop();
        return;
    }
    
    // Validate movement and leg positions
    validateMovement();
}