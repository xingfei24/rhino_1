#include "QuadrupedRobot.h"
#include <math.h>

// // 20241126添加
// // #include <Eigen/Dense>下面的可以代替它
// #include <ArduinoEigen.h>
// #include <chrono>

// using namespace Eigen;
// using namespace std::chrono;

QuadrupedRobot::QuadrupedRobot() : pwm(), mpu(), moving(true), currentGait(WALK), maxSpeed(1.0f), bodyHeight(230.0f),
                                   state(LEARNING)
{

    nn = new NeuralNetwork(14, 20, 8); // 14 inputs, 20 hidden, 8 outputs

    // gaitParams = {
    //     .stepHeight = 0.0f,
    //     .strideLength = 50.0f,
    //     .cycleTime = 1.0f,
    //     .phaseOffsets = {0.0f, 0.25f, 0.5f, 0.75f}};
    // 修改步态参数
    gaitParams = {

        .stepHeight = 100.0f,                       // 减小步高，使运动更稳定
        .strideLength = 50.0f,                      // 减小步长，提高稳定性
        .cycleTime = 1200.0f,                       // 调整周期时间
        .phaseOffsets = {0.0f, 0.5f, 0.25f, 0.75f}, // TROT步态的相位差
                                                    // phaseOffsets[0] = 0.0f;   // FRONT_RIGHT (右前腿)
                                                    // phaseOffsets[1] = 0.5f;   // FRONT_LEFT  (左前腿)
                                                    // phaseOffsets[2] = 0.25f;  // BACK_RIGHT  (右后腿)
                                                    // phaseOffsets[3] = 0.75f;  // BACK_LEFT   (左后腿)
        .dutyFactor = 0.65f                         // 调整占空比
    };

    initServoConfigs();
    initLegConfigs();
}

QuadrupedRobot::~QuadrupedRobot()
{
    delete nn;
}

void QuadrupedRobot::init()
{
    Wire.begin();
    delay(100); // 添加延迟确保I2C稳定

    pwm.begin();
    pwm.setPWMFreq(50); // Standard servo frequency
                        // 添加MPU6050初始化调试信息
    Serial.println("Initializing MPU6050...");
    if (!mpu.begin())
    {
        Serial.println("MPU6050 initialization failed!");
        while (1)
        {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    // 打印MPU6050配置
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange())
    {
    case MPU6050_RANGE_2_G:
        Serial.println("+-2G");
        break;
    case MPU6050_RANGE_4_G:
        Serial.println("+-4G");
        break;
    case MPU6050_RANGE_8_G:
        Serial.println("+-8G");
        break;
    case MPU6050_RANGE_16_G:
        Serial.println("+-16G");
        break;
    }
    calibrateSensors();
    stand();
}

void QuadrupedRobot::initServoConfigs()
{
    // Configure each servo with mirroring support
    for (int i = 0; i < TOTAL_SERVOS; i++)
    {
        servoConfigs[i] = {
            .pin = i,
            .minPulse = 102,
            .maxPulse = 508,
            .minAngle = 0.0f,
            .maxAngle = 180.0f,
            .offset = 0.0f,
            .reversed = false,         // 表示不反转伺服器的操作，如转动方向等
            .isMirrored = (i % 2 == 1) // 表示奇数索引的伺服器将作为偶数索引伺服器的镜像，以实现对称操作
        };
    }
}

void QuadrupedRobot::initLegConfigs()
{
    // Initialize leg configurations with proper mirroring
    const float X_OFFSET = 100.0f; // 原值是100
    const float Y_OFFSET = 12.0f;  // 原值是100

    legConfigs[FRONT_RIGHT] = {X_OFFSET, Y_OFFSET, -bodyHeight, false, 0.0f};
    legConfigs[FRONT_LEFT] = {X_OFFSET, -Y_OFFSET, -bodyHeight, true, 0.0f};
    legConfigs[BACK_RIGHT] = {-X_OFFSET, Y_OFFSET, -bodyHeight, false, 0.0f};
    legConfigs[BACK_LEFT] = {-X_OFFSET, -Y_OFFSET, -bodyHeight, true, 0.0f};

    for (int i = 0; i < NUM_LEGS; i++)
    {
        isGrounded[i] = true;
        lastContactTime[i] = millis();
    }
}

void QuadrupedRobot::update()
{
    updateMpuData();

    // Basic balance and movement updates
    if (moving)
    {
        balance();
    }

    validateMovement();
    optimizeEnergyUsage();
}

void QuadrupedRobot::calibrateSensors()
{
    Serial.println("Calibrating sensors...");
    float accelBias[3] = {0, 0, 0};
    float gyroBias[3] = {0, 0, 0};
    const int samples = 500;

    for (int i = 0; i < samples; i++)
    {
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
    for (int i = 0; i < 3; i++)
    {
        accelBias[i] /= samples;
        gyroBias[i] /= samples;
    }

    Serial.println("Calibration complete");
}

void QuadrupedRobot::setPIDGains(float kp, float ki, float kd)
{
    pidGains[0] = kp;
    pidGains[1] = ki;
    pidGains[2] = kd;
}

void QuadrupedRobot::setMaxSpeed(float speed)
{
    maxSpeed = constrain(speed, 0.0f, 1.0f);
}

void QuadrupedRobot::setStepHeight(float height)
{
    gaitParams.stepHeight = height;
}

void QuadrupedRobot::setBodyHeight(float height)
{
    bodyHeight = height;
    stand(); // Update standing position with new height
}

void QuadrupedRobot::setCycleTime(float time)
{
    gaitParams.cycleTime = time;
}

void QuadrupedRobot::setLegPhaseOffset(int legIndex, float offset)
{
    if (legIndex >= 0 && legIndex < NUM_LEGS)
    {
        gaitParams.phaseOffsets[legIndex] = offset;
    }
}

void QuadrupedRobot::walk(int cycles, float velocity, Direction direction)
{
    //     if (velocity > maxSpeed)
    //         velocity = maxSpeed;
    //     moving = true;

    //     unsigned long startTime = millis();
    //     int currentCycle = 0;

    //     while (moving && currentCycle < cycles)
    //     {
    //         unsigned long currentTime = millis();
    //         float phase = ((currentTime - startTime) % (unsigned long)gaitParams.cycleTime) / gaitParams.cycleTime;

    //         generateWalkingGait(phase, velocity, direction);

    //         float pitchCorrection, rollCorrection;
    //         calculateBalanceCorrection(pitchCorrection, rollCorrection);
    //         applyBalanceCorrection(pitchCorrection, rollCorrection);

    //         if (!isBalanced())
    //         {
    //             stabilize();
    //             break;
    //         }

    //         if (currentTime - startTime >= (currentCycle + 1) * gaitParams.cycleTime)
    //         {
    //             currentCycle++;
    //         }

    //         delay(10);
    //     }
    unsigned long startTime = millis();
    unsigned long currentTime;
    // float phase;

    float phase = ((currentTime - startTime) % (unsigned long)gaitParams.cycleTime) / gaitParams.cycleTime;

    if (velocity > maxSpeed)
    {
        velocity = maxSpeed;
    }

    moving = true;
    float currentVelocity = 0.0f;
    const float acceleration = velocity / 10.0f; // 逐渐加速

    int currentCycle = 0;

    while (moving && currentCycle < cycles)
    {
        currentTime = millis();
        // float phase = ((currentTime - startTime) % (unsigned long)gaitParams.cycleTime) / gaitParams.cycleTime;
        // 创建 Eigen::Vector3f 对象作为参数
        Eigen::Vector3f V_exp(0.0f, 0.0f, 0.0f);
        Eigen::Vector3f V_real(0.0f, 0.0f, 0.0f);

        // 20241126

        // 调用 generateWalkingGait 函数，提供正确的参数
        generateWalkingGait(phase, V_exp, V_real, direction);

        float pitchCorrection, rollCorrection;
        calculateBalanceCorrection(pitchCorrection, rollCorrection);
        applyBalanceCorrection(pitchCorrection, rollCorrection);
        // 逐渐增加速度
        if (currentVelocity < velocity)
        {
            currentVelocity += acceleration;
            if (currentVelocity > velocity)
            {
                currentVelocity = velocity;
            }
        }

        if (!isBalanced())
        {
            stabilize();
            break;
        }

        if (currentTime - startTime >= (currentCycle + 1) * gaitParams.cycleTime)
        {
            currentCycle++;
        }

        delay(10);
    }

    // 逐渐减速停止
    while (currentVelocity > 0 && moving)
    {
        currentVelocity -= acceleration;

        if (currentVelocity < 0)
        {
            currentVelocity = 0;
        }
        currentTime = millis();
        phase = ((currentTime - startTime) % (unsigned long)gaitParams.cycleTime) / gaitParams.cycleTime;
        // 重新声明并初始化 V_exp 和 V_real
        Eigen::Vector3f V_exp(0.0f, 0.0f, 0.0f);
        Eigen::Vector3f V_real(0.0f, 0.0f, 0.0f);
        generateWalkingGait(phase, V_exp, V_real, direction);
        delay(10);
    }
    moving = false;
    stand();
}

void QuadrupedRobot::stand()
{
    moving = false;

    for (int leg = 0; leg < NUM_LEGS; leg++)
    {
        float hipAngle, kneeAngle;
        calculateInverseKinematics(
            legConfigs[leg].x,
            legConfigs[leg].y,
            -bodyHeight,
            hipAngle,
            kneeAngle,
            legConfigs[leg].isMirrored);

        setServoAngle(leg * 2, hipAngle);
        setServoAngle(leg * 2 + 1, kneeAngle);
        isGrounded[leg] = true;
    }
}

void QuadrupedRobot::balance()
{
    float pitchCorrection, rollCorrection;
    calculateBalanceCorrection(pitchCorrection, rollCorrection);
    applyBalanceCorrection(pitchCorrection, rollCorrection);
}

void QuadrupedRobot::stabilize()
{
    updateMpuData();

    if (abs(mpuData.pitch) > 45.0f || abs(mpuData.roll) > 45.0f)
    {
        emergencyStop();
        return;
    }

    balance();

    if (isBalanced())
    {
        stand();
    }
}

void QuadrupedRobot::setGait(GaitType gait)
{
    currentGait = gait;
    gaitParams.dutyFactor = 0.65;
    switch (gait)
    {
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

void QuadrupedRobot::jump(float height, float duration)
{
    // Prepare for jump
    crouch(bodyHeight * 0.7f);
    delay(200);

    // Jump motion
    for (int leg = 0; leg < NUM_LEGS; leg++)
    {
        float hipAngle, kneeAngle;
        calculateInverseKinematics(
            legConfigs[leg].x,
            legConfigs[leg].y,
            -bodyHeight + height,
            hipAngle,
            kneeAngle,
            legConfigs[leg].isMirrored);

        setServoAngle(leg * 2, hipAngle);
        setServoAngle(leg * 2 + 1, kneeAngle);
    }

    delay(duration);

    // Land
    stand();
}

void QuadrupedRobot::turn(float angle)
{
    const float turnRadius = sqrt(
        legConfigs[0].x * legConfigs[0].x +
        legConfigs[0].y * legConfigs[0].y);

    for (int leg = 0; leg < NUM_LEGS; leg++)
    {
        float currentAngle = atan2(legConfigs[leg].y, legConfigs[leg].x) * 180.0 / M_PI;
        float newAngle = currentAngle + angle * 180.0 / M_PI;

        float hipAngle, kneeAngle;
        calculateInverseKinematics(
            cos(newAngle) * turnRadius,
            sin(newAngle) * turnRadius,
            -bodyHeight,
            hipAngle,
            kneeAngle,
            legConfigs[leg].isMirrored);

        setServoAngle(leg * 2, hipAngle);
        setServoAngle(leg * 2 + 1, kneeAngle);
    }
}

void QuadrupedRobot::crouch(float height)
{
    for (int leg = 0; leg < NUM_LEGS; leg++)
    {
        float hipAngle, kneeAngle;
        calculateInverseKinematics(
            legConfigs[leg].x,
            legConfigs[leg].y,
            -height,
            hipAngle,
            kneeAngle,
            legConfigs[leg].isMirrored);

        setServoAngle(leg * 2, hipAngle);
        setServoAngle(leg * 2 + 1, kneeAngle);
    }
}

void QuadrupedRobot::stretch()
{
    // Stretch sequence
    for (int leg = 0; leg < NUM_LEGS; leg++)
    {
        // Lift leg
        float hipAngle, kneeAngle;
        calculateInverseKinematics(
            legConfigs[leg].x,
            legConfigs[leg].y,
            // -bodyHeight + 50.0f,
            -bodyHeight,
            hipAngle,
            kneeAngle,
            legConfigs[leg].isMirrored);

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
            legConfigs[leg].isMirrored);

        setServoAngle(leg * 2, hipAngle);
        setServoAngle(leg * 2 + 1, kneeAngle);

        delay(500);
    }
}

bool QuadrupedRobot::isBalanced() const
{
    const float threshold = 5.0f; // degrees
    return abs(mpuData.pitch) < threshold && abs(mpuData.roll) < threshold;
}

bool QuadrupedRobot::isMoving() const
{
    return moving;
}

float QuadrupedRobot::getBatteryLevel() const
{
    // Implement battery level reading
    return 0.0f; // Placeholder
}

const MpuData &QuadrupedRobot::getSensorData() const
{
    return mpuData;
}

void QuadrupedRobot::emergencyStop()
{
    moving = false;
    state = EMERGENCY;
    stand();
    Serial.println("Emergency stop triggered!");
}

void QuadrupedRobot::updateMpuData()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // 添加调试输出
    if (debugMode)
    { // 20241115 22:25:24

        Serial.println("Raw MPU Data:");
        Serial.print("Accel: ");
        Serial.print(a.acceleration.x);
        Serial.print(", ");
        Serial.print(a.acceleration.y);
        Serial.print(", ");
        Serial.println(a.acceleration.z);
    }
    mpuData.accel_x = a.acceleration.x;
    mpuData.accel_y = a.acceleration.y;
    mpuData.accel_z = a.acceleration.z;

    mpuData.gyro_x = g.gyro.x;
    mpuData.gyro_y = g.gyro.y;
    mpuData.gyro_z = g.gyro.z;

    mpuData.temp = temp.temperature;

    mpuData.calculateAngles();
}

void QuadrupedRobot::calculateBalanceCorrection(float &pitchCorrection, float &rollCorrection)
{
    const float dt = 0.01f; // Fixed time step for stability

    float pitch = mpuData.pitch;
    float roll = mpuData.roll;

    // Simple P controller for stability
    pitchCorrection = pidGains[0] * pitch;
    rollCorrection = pidGains[0] * roll;
}

void QuadrupedRobot::applyBalanceCorrection(float pitchCorrection, float rollCorrection)
{
    for (int leg = 0; leg < NUM_LEGS; leg++)
    {
        float z = -bodyHeight;

        // Apply pitch correction (front/back)
        if (leg < 2)
            z += pitchCorrection * 20.0f;
        else
            z -= pitchCorrection * 20.0f;

        // Apply roll correction (left/right)
        if (leg % 2)
            z += rollCorrection * 20.0f;
        else
            z -= rollCorrection * 20.0f;

        float hipAngle, kneeAngle;
        calculateInverseKinematics(
            legConfigs[leg].x,
            legConfigs[leg].y,
            z,
            hipAngle,
            kneeAngle,
            legConfigs[leg].isMirrored);

        setServoAngle(leg * 2, hipAngle);
        setServoAngle(leg * 2 + 1, kneeAngle);
    }
}

// void QuadrupedRobot::generateWalkingGait(float phase, float velocity, Direction direction)
// {
//     float stride = velocity * gaitParams.strideLength;

//     for (int leg = 0; leg < NUM_LEGS; leg++)
//     {
//         float legPhase = fmod(phase + gaitParams.phaseOffsets[leg], 1.0f);
//         float x = legConfigs[leg].x;
//         float y = legConfigs[leg].y;
//         float z = -bodyHeight;
//         // 添加调试输出
//         if (debugMode)
//         {
//             Serial.print("Leg ");
//             Serial.print(leg);
//             Serial.print(" Phase: ");
//             Serial.println(legPhase);
//         }
//         // 增加后腿的运动幅度
//         float amplitudeFactor = (leg >= 2) ? 1.3f : 1.0f; // 后腿增加30%的运动幅度
//         if (legPhase < 0.5f)                              // 支撑相
//         {
//             // Stance phase
//             // float stanceX = cos(legPhase * M_PI * 2) * stride;
//             // float stanceY = 0;
//             // 将支撑相改为线性运动
//             float stancePhase = legPhase * 2.0f;                  // 归一化到0-1范围
//                                                                   // float stanceX = (0.5f - stancePhase) * stride; // 线性移动
//             float stanceX = (0.5f - stancePhase) * stride * 1.2f; // 增加1.2倍的运动幅度

//             float stanceY = 0;
//             // 根据方向调整运动
//             switch (direction)
//             {
//             case FORWARD:
//                 stanceX = +stanceX;
//                 break;
//             case BACKWARD:
//                 stanceX = -stanceX;
//                 break;
//             case LEFT:
//                 stanceY = -stanceX;
//                 stanceX = 0;
//                 break;
//             case RIGHT:
//                 stanceY = +stanceX;
//                 stanceX = 0;
//                 break;
//             default: // FORWARD
//                 break;
//             }

//             x += stanceX;
//             y += stanceY;
//             isGrounded[leg] = true;
//             lastContactTime[leg] = millis();
//         }
//         //         else
//         //         {
//         //             // Swing phase
//         //             // z = -bodyHeight + sin((legPhase - 0.5f) * M_PI * 2) * gaitParams.stepHeight;
//         //             // isGrounded[leg] = false;
//         //             // 贝塞尔曲线swing
//         //             float swingPhase = legPhase;
//         //             float t = swingPhase;
//         //             float oneMinusT = 1.0 - t;

//         //             x = direction * gaitParams.strideLength * (-0.5 + swingPhase);

//         //             float h0 = 0, h1 = gaitParams.stepHeight;
//         //             float h2 = gaitParams.stepHeight, h3 = 0;

//         //             z = h0 * pow(oneMinusT, 3) +
//         //                 3 * h1 * pow(oneMinusT, 2) * t +
//         //                 3 * h2 * oneMinusT * pow(t, 2) +
//         //                 h3 * pow(t, 3);
//         //         }
//         //         z *= 0.8 + velocity * 0.1;
//         //         y = 0;
//         //         isGrounded[leg] = false;

//         //         float hipAngle, kneeAngle;
//         //         calculateInverseKinematics(x, y, z, hipAngle, kneeAngle, legConfigs[leg].isMirrored);

//         //         setServoAngle(leg * 2, hipAngle);
//         //         setServoAngle(leg * 2 + 1, kneeAngle);
//         //     }
//         // }
//         else // 摆动相
//         {
//             float swingPhase = (legPhase - 0.5f) * 2.0f; // 归一化到0-1范围

//             // 使用正弦曲线生成平滑的摆动轨迹
//             // float swingX = (-0.5f + swingPhase) * stride;
//             float swingX = (-0.5f + swingPhase) * stride * amplitudeFactor; // 增加1.2倍的运动幅度
//                                                                             // 后腿抬得更高
//             float heightFactor = (leg >= 2) ? 1.4f : 1.0f;                  // 后腿抬高40%
//             // float swingHeight = sin(swingPhase * M_PI) * gaitParams.stepHeight;
//             float swingHeight = sin(swingPhase * M_PI) * gaitParams.stepHeight * heightFactor; // 增加1.5倍的抬腿高度

//             // 根据方向调整运动
//             switch (direction)
//             {
//             case FORWARD:
//                 x += swingX;
//                 z += swingHeight;
//                 break;
//             case BACKWARD:
//                 x -= swingX;
//                 z += swingHeight;
//                 break;
//             case LEFT:
//                 y -= swingX;
//                 z += swingHeight;
//                 break;
//             case RIGHT:
//                 y += swingX;
//                 z += swingHeight;
//                 break;
//             }
//             isGrounded[leg] = false;
//         }

//         // 添加速度相关的高度调整
//         // z *= (0.9f + velocity * 0.2f);
//         z *= (0.85f + velocity * 0.3f); // 增加速度对高度的影响

//         // 添加调试输出
//         if (debugMode)
//         {
//             Serial.print("Leg ");
//             Serial.print(leg);
//             Serial.print(" - Phase: ");
//             Serial.print(legPhase);
//             Serial.print(" X: ");
//             Serial.print(x);
//             Serial.print(" Y: ");
//             Serial.print(y);
//             Serial.print(" Z: ");
//             Serial.println(z);
//         }
//         // 计算并设置关节角度
//         float hipAngle, kneeAngle;
//         calculateInverseKinematics(x, y, z, hipAngle, kneeAngle, legConfigs[leg].isMirrored);

//         setServoAngle(leg * 2, hipAngle);
//         setServoAngle(leg * 2 + 1, kneeAngle);
//     }
// }
// void QuadrupedRobot::generateWalkingGait(float phase, float velocity, Direction direction)
// {
//     float stride = velocity * gaitParams.strideLength;

//     for (int leg = 0; leg < NUM_LEGS; leg++)
//     {
//         float legPhase = fmod(phase + gaitParams.phaseOffsets[leg], 1.0f);
//         float x = legConfigs[leg].x;
//         float y = legConfigs[leg].y;
//         float z = -bodyHeight;

//         // 调试输出
//         if (debugMode)
//         {
//             Serial.print("Leg ");
//             Serial.print(leg);
//             Serial.print(" Phase: ");
//             Serial.println(legPhase);
//         }
//         float dutyCycle = gaitParams.dutyFactor;
//         if (legPhase < dutyCycle) // 支撑相
//         {
//             float stancePhase = legPhase * 2.0f;
//             float stanceX = stride * (1 - cos(PI * stancePhase)) / 2; // 使用正弦曲线创建平滑位移
//             float stanceY = 0;

//             // 根据方向调整运动
//             switch (direction)
//             {
//             case FORWARD:
//                 x += stanceX;
//                 break;
//             case BACKWARD:
//                 x -= stanceX;
//                 break;
//             case LEFT:
//                 y -= stanceX;
//                 break;
//             case RIGHT:
//                 y += stanceX;
//                 break;
//             }

//             isGrounded[leg] = true;
//             lastContactTime[leg] = millis();
//         }
//         else // 摆动相
//         {
//             // float swingPhase = (legPhase - 0.5f) * 2.0f;
//             float swingPhase = (legPhase - dutyCycle) / (1.0f - dutyCycle);
//             float amplitudeFactor = (leg >= 2) ? 1.3f : 1.0f;
//             float heightFactor = (leg >= 2) ? 1.4f : 1.0f;

//             // 使用改进的轨迹生成方法
//             float swingX = stride * amplitudeFactor * (1 - cos(PI * swingPhase)) / 2;
//             float swingHeight = gaitParams.stepHeight * heightFactor *
//                                 (16 * pow(swingPhase, 3) - 32 * pow(swingPhase, 4) + 16 * pow(swingPhase, 5));

//             // 根据方向调整运动
//             switch (direction)
//             {
//             case FORWARD:
//                 x += swingX;
//                 z += swingHeight;
//                 break;
//             case BACKWARD:
//                 x -= swingX;
//                 z += swingHeight;
//                 break;
//             case LEFT:
//                 y -= swingX;
//                 z += swingHeight;
//                 break;
//             case RIGHT:
//                 y += swingX;
//                 z += swingHeight;
//                 break;
//             }

//             isGrounded[leg] = false;
//         }

//         // 速度相关的高度调整
//         z *= (0.85f + velocity * 0.3f);

//         // 调试输出
//         if (debugMode)
//         {
//             Serial.print("Leg ");
//             Serial.print(leg);
//             Serial.print(" - Phase: ");
//             Serial.print(legPhase);
//             Serial.print(" X: ");
//             Serial.print(x);
//             Serial.print(" Y: ");
//             Serial.print(y);
//             Serial.print(" Z: ");
//             Serial.println(z);
//         }

//         // 计算并设置关节角度
//         float hipAngle, kneeAngle;
//         calculateInverseKinematics(x, y, z, hipAngle, kneeAngle, legConfigs[leg].isMirrored);

//         setServoAngle(leg * 2, hipAngle);
//         setServoAngle(leg * 2 + 1, kneeAngle);
//     }
// }
// 修正后的 generateWalkingGait 函数（可以运行20241126）
// void QuadrupedRobot::generateWalkingGait(float phase, float velocity, Direction direction)
// {
//     // 使用类成员变量时添加 this 指针
//     float stride = velocity * this->gaitParams.strideLength;
//     float Ts = this->gaitParams.cycleTime;
//     float lamuda = this->gaitParams.dutyFactor;
//     float leg_raise = this->gaitParams.stepHeight;
//     float K_bx = 1.0f;
//     float K_by = 1.0f;

//     for (int leg = 0; leg < NUM_LEGS; leg++)
//     {
//         float legPhase = fmod(phase + this->gaitParams.phaseOffsets[leg], 1.0f);
//         float x = this->legConfigs[leg].x;
//         float y = this->legConfigs[leg].y;
//         float z = -this->bodyHeight;

//         // 调试输出
//         if (this->debugMode)
//         {
//             Serial.print("Leg ");
//             Serial.print(leg);
//             Serial.print(" Phase: ");
//             Serial.println(legPhase);
//         }
//         float x_start, y_start, z_start; // 声明变量
//         float x_end, y_end;              // 声明变量

//         if (legPhase == 0.0f)
//         {
//             // 计算起始点
//             float x_start = x;
//             float y_start = y;
//             float z_start = z;

//             // 计算落点
//             float x_end = 0.5f * velocity * lamuda * Ts - K_bx * (velocity - velocity);
//             float y_end = 0.5f * velocity * lamuda * Ts - K_by * (velocity - velocity);
//         }
//         else if (legPhase < lamuda)
//         {
//             // 支撑相
//             float segema = (2 * PI * legPhase) / lamuda;
//             // float x_start = 0.0f; // 这里需要定义起始点，假设为 0.0f
//             // float y_start = 0.0f;
//             // float x_end = 0.0f;
//             // float y_end = 0.0f;
//             x = (x_end - x_start) * ((segema - sin(segema)) / (2 * PI)) + x_start;
//             y = (y_end - y_start) * ((segema - sin(segema)) / (2 * PI)) + y_start;
//             z = z_start;

//             this->isGrounded[leg] = true;
//             this->lastContactTime[leg] = millis();
//         }
//         else
//         {
//             // 摆动相
//             float swingPhase = (legPhase - lamuda) / (1.0f - lamuda);
//             float segema = (2 * PI * swingPhase);
//             float amplitudeFactor = (leg >= 2) ? 1.3f : 1.0f;
//             float heightFactor = (leg >= 2) ? 1.4f : 1.0f;

//             // float x_start = 0.0f; // 这里需要定义起始点，假设为 0.0f
//             // float y_start = 0.0f;
//             // float x_end = 0.0f;
//             // float y_end = 0.0f;
//             // x = (x_end - x_start) * (1 - ((legPhase - lamuda) / (1 - lamuda)));
//             // y = (y_end - y_start) * (1 - ((legPhase - lamuda) / (1 - lamuda)));
//             // z = leg_raise * ((1 - cos(segema)) / 2) + z_start;

//             // this->isGrounded[leg] = false;
//             // 使用改进的轨迹生成方法
//             float swingX = stride * amplitudeFactor * (1 - cos(PI * swingPhase)) / 2;
//             float swingHeight = gaitParams.stepHeight * heightFactor *
//                                 (16 * pow(swingPhase, 3) - 32 * pow(swingPhase, 4) + 16 * pow(swingPhase, 5));

//             // 计算起始点
//             x_start = x;
//             y_start = y;
//             z_start = z;

//             // 计算落点
//             x_end = x + swingX;
//             y_end = y + swingX;

//             x = x_start + swingX * swingPhase;
//             y = y_start + swingX * swingPhase;
//             z = z_start + swingHeight;

//             isGrounded[leg] = false;
//         }

//         // 根据方向调整运动
//         switch (direction)
//         {
//         case FORWARD:
//             break;
//         case BACKWARD:
//             x = -x;
//             break;
//         case LEFT:
//         {
//             float temp = x;
//             x = -y;
//             y = temp;
//             break;
//         }
//         case RIGHT:
//         {
//             float temp = x; // 声明 temp 变量
//             temp = x;
//             x = y;
//             y = -temp;
//             break;
//         }
//         }

//         // 调试输出
//         if (this->debugMode)
//         {
//             Serial.print("Leg ");
//             Serial.print(leg);
//             Serial.print(" - Phase: ");
//             Serial.print(legPhase);
//             Serial.print(" X: ");
//             Serial.print(x);
//             Serial.print(" Y: ");
//             Serial.print(y);
//             Serial.print(" Z: ");
//             Serial.println(z);
//         }

//         // 计算并设置关节角度
//         float hipAngle, kneeAngle;
//         this->calculateInverseKinematics(x, y, z, hipAngle, kneeAngle, this->legConfigs[leg].isMirrored);

//         this->setServoAngle(leg * 2, hipAngle);
//         this->setServoAngle(leg * 2 + 1, kneeAngle);
//     }
// }
void QuadrupedRobot::generateWalkingGait(float phase, Vector3f &V_exp, Vector3f &V_real, Direction direction)
{
    // float stride = velocity * gaitParams.strideLength;
    float stride = this->velocity * gaitParams.strideLength;
    float Ts = gaitParams.cycleTime;
    float lamuda = gaitParams.dutyFactor;
    float leg_raise = gaitParams.stepHeight;
    float K_bx = 1.0f;
    float K_by = 1.0f;
    static float t = phase;
    static auto t_last = high_resolution_clock::now();
    for (int leg = 0; leg < NUM_LEGS; leg++)
    {
        float legPhase = fmod(t + gaitParams.phaseOffsets[leg], Ts);
        float x = legConfigs[leg].x;
        float y = legConfigs[leg].y;
        float z = -bodyHeight;

        // 调试输出
        if (debugMode)
        {
            Serial.print("Leg ");
            Serial.print(leg);
            Serial.print(" Phase: ");
            Serial.println(legPhase);
        }
        float x_start, y_start, z_start; // 声明变量
        float x_end, y_end;              // 声明变量

        if (legPhase == 0.0f)
        {
            // 计算起始点
            float x_start = x;
            float y_start = y;
            float z_start = z;

            // // 计算落点
            // x_end = 0.5f * V_real(0) * lamuda * Ts - K_bx * (V_exp(0) - V_real(0));
            // y_end = 0.5f * V_real(1) * lamuda * Ts - K_by * (V_exp(1) - V_real(1));
             float V_diff_x = V_exp(0) - V_real(0);
            float V_diff_y = V_exp(1) - V_real(1);
            // 限制 V_diff_x 和 V_diff_y 的范围
            const float V_diff_max = 10.0f; // 假设最大差值为 10.0f
            if (V_diff_x > V_diff_max)
            {
                V_diff_x = V_diff_max;
            }
            else if (V_diff_x < -V_diff_max)
            {
                V_diff_x = -V_diff_max;
            }
            if (V_diff_y > V_diff_max)
            {
                V_diff_y = V_diff_max;
            }
            else if (V_diff_y < -V_diff_max)
            {
                V_diff_y = -V_diff_max;
            }
            // 计算落点
            x_end = 0.5f * V_real(0) * lamuda * Ts - K_bx * V_diff_x;
            y_end = 0.5f * V_real(1) * lamuda * Ts - K_by * V_diff_y;
        }
        if (legPhase < lamuda * Ts)
        {
            // 支撑相
            float segema = (2 * PI * legPhase) / (lamuda * Ts);
            x = (x_end - x_start) * ((segema - sin(segema)) / (2 * PI)) + x_start;
            y = (y_end - y_start) * ((segema - sin(segema)) / (2 * PI)) + y_start;
            z = z_start;

            isGrounded[leg] = true;
            lastContactTime[leg] = millis();
        }
        else
        {
            // 摆动相
            float swingPhase = (legPhase - lamuda * Ts) / ((1 - lamuda) * Ts);
            float segema = 2 * PI * swingPhase;
            x = (x_end - x_start) * (1 - swingPhase) + x_start;
            y = (y_end - y_start) * (1 - swingPhase) + y_start;
            z = leg_raise * ((1 - cos(segema)) / 2) + z_start;

            isGrounded[leg] = false;
        }

        // 根据方向调整运动
        switch (direction)
        {
        case FORWARD:
            break;
        case BACKWARD:
            x = -x;
            break;
        case LEFT:
        {
            float temp = x;
            x = -y;
            y = temp;
            break;
        }
        case RIGHT:
        {
            float temp = x; // 声明 temp 变量
            temp = x;
            x = y;
            y = -temp;
            break;
        }
        }

        // 调试输出

        if (debugMode)
        {
            Serial.print("Leg ");
            Serial.print(leg);
            Serial.print(" - Phase: ");
            Serial.print(legPhase);
            Serial.print(" X: ");
            Serial.print(x);
            Serial.print(" Y: ");
            Serial.print(y);
            Serial.print(" Z: ");
            Serial.println(z);
        }

        // 计算并设置关节角度
        float hipAngle, kneeAngle;
        this->calculateInverseKinematics(x, y, z, hipAngle, kneeAngle, this->legConfigs[leg].isMirrored);

        this->setServoAngle(leg * 2, hipAngle);
        this->setServoAngle(leg * 2 + 1, kneeAngle);
    }
    // 更新时间
    auto t_now = high_resolution_clock::now();
    duration<float> time_span = duration_cast<duration<float>>(t_now - t_last);
    float maxTimeStep = 0.1f; // 最大时间步长
    if (time_span.count() > maxTimeStep)
    {
        time_span = duration<float>(maxTimeStep);
    }
    t += time_span.count();
    t_last = t_now;

    // 周期管理
    if (t > Ts)
    {
        t -= Ts;
    }
}
void QuadrupedRobot::calculateLegForces()
{
    // Implement force calculation for each leg
    // This is a placeholder for actual force sensor implementation
}

// void QuadrupedRobot::calculateInverseKinematics(float x, float y, float z,
//                                                 float &hipAngle, float &kneeAngle,
//                                                 bool isMirrored)
// {
//     const float L1 = 120.0f; // Upper leg length
//     const float L2 = 120.0f; // Lower leg length

//     // Mirror the Y coordinate for left legs
//     if (isMirrored)
//     {
//         y = -y;
//     }

//     // Calculate leg length and angle in the x-y plane
//     float L = sqrt(x * x + y * y);
//     float gamma = atan2(y, x);

//     // Calculate leg length in the x-z plane
//     float D = sqrt(L * L + z * z);

//     // Check if position is reachable
//     if (D > (L1 + L2) * 0.99f)
//     {
//         float scale = (L1 + L2) * 0.99f / D;
//         x *= scale;
//         y *= scale;
//         z *= scale;
//         D = (L1 + L2) * 0.99f;
//     }

//     // Calculate joint angles using cosine law
//     float alpha = acos((L1 * L1 + D * D - L2 * L2) / (2 * L1 * D)) + atan2(z, L);
//     float beta = acos((L1 * L1 + L2 * L2 - D * D) / (2 * L1 * L2));

//     // Convert to degrees and apply mirroring if needed
//     hipAngle = clampAngle(gamma * 180.0f / M_PI, -85.0f, 85.0f);//20241118修改
//         // hipAngle = clampAngle(180-gamma * 180.0f / M_PI, 85.0f, 135.0f);//20241118修改

//     hipAngle = hipAngle + 90.0f; // 转换到0-180度范围

//     kneeAngle = clampAngle((M_PI - beta) * 180.0f / M_PI, 10.0f, 135.0f);

//     if (isMirrored)
//     {
//         // hipAngle = mirrorAngle(hipAngle, true);
//         hipAngle = 180.0f - hipAngle; // 修改镜像逻辑
//     }
// }

void QuadrupedRobot::calculateInverseKinematics(float x, float y, float z,
                                                float &hipAngle, float &kneeAngle,
                                                bool isMirrored)
{
    const float L1 = 120.0f; // 大腿长度
    const float L2 = 120.0f; // 小腿长度

    // 处理镜像腿
    if (isMirrored)
    {
        y = -y;
    }

    // 计算腿部水平投影长度
    float horizontalLength = sqrt(x * x + y * y);

    // 计算腿部总长度
    float totalLength = sqrt(horizontalLength * horizontalLength + z * z);

    // 检查位置是否可达
    if (totalLength > (L1 + L2) * 0.99f)
    {
        float scale = (L1 + L2) * 0.99f / totalLength;
        x *= scale;
        y *= scale;
        z *= scale;
        totalLength = (L1 + L2) * 0.99f;
    }

    // 髋关节角度（水平面）
    float hipAngleRad = atan2(y, x);

    // 腿部倾斜角度
    float legAngleRad = atan2(z, horizontalLength);

    // 使用余弦定理计算膝关节角度

    float cosKneeAngle = (L1 * L1 + L2 * L2 - totalLength * totalLength) / (2 * L1 * L2);
    float kneeAngleRad = acos(cosKneeAngle);

    // 角度转换和限幅
    hipAngle = clampAngle(hipAngleRad * 180.0f / M_PI, 75.0f, 150.0f);
    kneeAngle = clampAngle((M_PI - kneeAngleRad) * 180.0f / M_PI, 120.0f, 145.0f);

    // Serial.print(" hipAngle: ");
    // Serial.print(hipAngle);
    // Serial.print(" - kneeAngle: ");
    // Serial.print(kneeAngle);

    // 镜像处理
    if (isMirrored)
    {
        hipAngle = 180.0f - hipAngle;
    }
}
void QuadrupedRobot::setServoAngle(int servoIndex, float angle)
{ // 20241118修改
  //  if (servoIndex >= TOTAL_SERVOS)
  //      return;

    // const ServoConfig &config = servoConfigs[servoIndex];
    // float adjustedAngle = angle + config.offset;

    // if (config.reversed)
    // {
    //     //  adjustedAngle = -adjustedAngle;//原代码
    //     adjustedAngle = 180.0f - adjustedAngle; // 修改反转逻辑
    // }

    // if (config.isMirrored)
    // {
    //     // adjustedAngle = mirrorAngle(adjustedAngle, true);// 原代码
    //     adjustedAngle = 180.0f - adjustedAngle; // 修改镜像逻辑
    // }

    // adjustedAngle = clampAngle(adjustedAngle, config.minAngle, config.maxAngle);
    // // float pulse = map(adjustedAngle, -90, 90, config.minPulse, config.maxPulse);
    // float pulse = map(adjustedAngle, 0, 180, config.minPulse, config.maxPulse); // 修改映射范围

    // pwm.setPWM(config.pin, 0, (int)pulse);
    // currentLegAngles[servoIndex] = angle;
    if (servoIndex < 0 || servoIndex >= TOTAL_SERVOS)
    {
        Serial.println("Invalid servo index");
        return;
    }

    ServoConfig &config = servoConfigs[servoIndex];
    if (angle < config.minAngle || angle > config.maxAngle)
    {
        Serial.println("Servo angle limit exceeded");
        return;
    }

    float adjustedAngle = angle + config.offset;

    if (config.reversed)
    {
        adjustedAngle = 180.0f - adjustedAngle; // 修改反转逻辑
    }

    if (config.isMirrored)
    {
        adjustedAngle = 180.0f - adjustedAngle; // 修改镜像逻辑
    }

    adjustedAngle = clampAngle(adjustedAngle, config.minAngle, config.maxAngle);
    float pulse = map(adjustedAngle, 0, 180, config.minPulse, config.maxPulse); // 修改映射范围

    // pwm.setServoPulse(servoIndex, pulse);
    pwm.setPWM(servoIndex, 0, (int)pulse);

    currentLegAngles[servoIndex] = angle;
}

float QuadrupedRobot::clampAngle(float angle, float min, float max)
{
    return fmax(min, fmin(max, angle));
}

float QuadrupedRobot::mirrorAngle(float angle, bool isMirrored)
{
    return isMirrored ? -angle : angle;
}

void QuadrupedRobot::validateMovement()
{
    const float MAX_EXTENSION = 300.0f; // 增加最大伸展限制
    const float MIN_EXTENSION = 0.0f;   // 添加最小伸展限制
    for (int i = 0; i < NUM_LEGS; i++)
    {
        // Check leg extension limits
        float x = legConfigs[i].x;
        float y = legConfigs[i].y;
        float z = legConfigs[i].z;

        float extension = sqrt(x * x + y * y + z * z);
        if (extension > 100.0f) // 原值是100
        {                       // Maximum leg extension
            emergencyStop();
            Serial.println("Leg extension limit exceeded!");
            return;
        }
        // 检查腿部伸展是否在合理范围内
        if (extension > MAX_EXTENSION || extension < MIN_EXTENSION)
        {
            Serial.print("Leg ");
            Serial.print(i);
            Serial.print(" extension: ");
            Serial.println(extension);
            // emergencyStop();
            // Serial.println("Leg extension limit exceeded!");
            return;
        }
        // Check servo angle limits
        // if (abs(currentLegAngles[i * 2]) > 75.0f ||
        //     currentLegAngles[i * 2 + 1] < 15.0f ||
        //     currentLegAngles[i * 2 + 1] > 135.0f)
        // 修改角度检查范围
        float hipAngle = currentLegAngles[i * 2];
        float kneeAngle = currentLegAngles[i * 2 + 1];

        if (hipAngle < 15.0f || hipAngle > 165.0f || // 髋关节限制（对应原来的±75度）
            kneeAngle < 15.0f || kneeAngle > 165.0f) // 膝关节限制保持不变

        {

            if (debugMode)
            {
                Serial.print("Leg ");
                Serial.print(i);
                Serial.print(" angles: Hip=");
                Serial.print(currentLegAngles[i * 2]);
                Serial.print(", Knee=");
                Serial.println(currentLegAngles[i * 2 + 1]);
            }
            Serial.print("Servo angle limit exceeded on leg ");
            Serial.println(i);
            emergencyStop();
            Serial.println("Servo angle limit exceeded!");
            Serial.println("Servo angle limit exceeded on leg " + String(i));

            return;
        }
    }
}

void QuadrupedRobot::optimizeEnergyUsage()
{
    // Implement power optimization strategies
    // This is a placeholder for actual power management
}

void QuadrupedRobot::monitorHealth()
{
    // Check orientation limits
    if (abs(mpuData.pitch) > 45.0f || abs(mpuData.roll) > 45.0f)
    {
        // emergencyStop();
        return;
    }

    // Validate movement and leg positions
    validateMovement();
}
void QuadrupedRobot::setMotorPWM(int motorIndex, int pwmValue)
{
    if (motorIndex < 0 || motorIndex >= NUM_MOTORS)
    {
        return; // 无效的电机索引
    }

    // 限制PWM值在有效范围内（假设PWM范围是0-1023）
    pwmValue = constrain(pwmValue, 102, 508);

    // 存储PWM值
    motorPWMs[motorIndex] = pwmValue;

    // 这里需要根据您的硬件实现来实际设置PWM
    // 例如，如果使用Servo库：
    // servos[motorIndex].writeMicroseconds(pwmValue);

    // 或者如果使用PWM引脚：
    // analogWrite(motorPins[motorIndex], pwmValue);
}
// void QuadrupedRobot::setDebugMode(bool enable)
// {
//     debugMode = enable;
//     if (enable) {
//         Serial.println("Debug mode enabled");
//     }
// }