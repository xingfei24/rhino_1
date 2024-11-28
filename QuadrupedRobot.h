#ifndef QUADRUPED_ROBOT_H
#define QUADRUPED_ROBOT_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include "NeuralNetwork.h"
#include "DataCollector.h"
#include "MpuData.h"

// 20241126添加
#include <ArduinoEigen.h>
#include <chrono>
using namespace Eigen;
using namespace std::chrono;
enum Direction
{
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

enum GaitType
{
    TROT,
    WALK,
    GALLOP,
    BOUND
};

enum LegPosition
{
    FRONT_RIGHT = 0,
    FRONT_LEFT = 1,
    BACK_RIGHT = 2,
    BACK_LEFT = 3
};

enum RobotState
{
    LEARNING,
    OPERATING,
    EMERGENCY
};

struct GaitParameters
{
    float stepHeight;
    float strideLength;
    float cycleTime;
    float phaseOffsets[4];
    float dutyFactor;
};

struct LegConfig
{
    float x, y, z;
    bool isMirrored;
    float baseAngle; // Base rotation for mirroring
};

struct ServoConfig
{
    int pin;
    int minPulse;
    int maxPulse;
    float minAngle;
    float maxAngle;
    float offset;
    bool reversed;
    bool isMirrored; // For left-right mirroring
};

class QuadrupedRobot
{
public:
    QuadrupedRobot();
    ~QuadrupedRobot();

    // Initialization and setup
    void init();
    void calibrateSensors();
    void setPIDGains(float kp, float ki, float kd);
    void setMaxSpeed(float speed);
    void setStepHeight(float height);
    void setBodyHeight(float height);
    void setCycleTime(float time);
    void setLegPhaseOffset(int legIndex, float offset);

    // Main control functions
    void update();
    void monitorHealth();

    // Movement control
    void walk(int cycles, float velocity = 0.75f, Direction direction = FORWARD);
    void generateWalkingGait(float phase, Eigen::Vector3f &V_exp, Eigen::Vector3f &V_real, Direction direction);

    void stand();
    void balance();
    void stabilize();
    void setGait(GaitType gait);
    void jump(float height, float duration);
    void turn(float angle);
    void crouch(float height);
    void stretch();

    // State management
    bool isBalanced() const;
    bool isMoving() const;
    float getBatteryLevel() const;
    const MpuData &getSensorData() const;
    void emergencyStop();
    // 设置指定电机的PWM值
    void setMotorPWM(int motorIndex, int pwmValue);
    void setDebugMode(bool enable)
    {
        debugMode = enable;
    }

private:
    static const int NUM_LEGS = 4;
    static const int SERVOS_PER_LEG = 2;
    static const int TOTAL_SERVOS = NUM_LEGS * SERVOS_PER_LEG;
    static const int NUM_MOTORS = 8; // 总电机数量
    int motorPWMs[NUM_MOTORS];       // 存储每个电机的PWM值
    bool debugMode = false;
    bool moving; // 判断是否在移动
                 // 假设 velocity 是成员变量
    float velocity;
    // Hardware components
    Adafruit_PWMServoDriver pwm;
    Adafruit_MPU6050 mpu;
    NeuralNetwork *nn;
    DataCollector dataCollector;

    // Configuration
    ServoConfig servoConfigs[TOTAL_SERVOS];
    LegConfig legConfigs[NUM_LEGS];
    GaitParameters gaitParams;
    float pidGains[3]; // Kp, Ki, Kd
    float maxSpeed;
    float bodyHeight;

    // State variables
    RobotState state;
    float currentLegAngles[TOTAL_SERVOS];
    float targetAngles[TOTAL_SERVOS];
    bool isGrounded[NUM_LEGS];
    unsigned long lastContactTime[NUM_LEGS];
    GaitType currentGait;
    MpuData mpuData;

    // Methods
    void initServoConfigs();
    void initLegConfigs();
    void updateMpuData();
    void calculateOrientation(float dt);
    void calculateBalanceCorrection(float &pitchCorrection, float &rollCorrection);
    void applyBalanceCorrection(float pitchCorrection, float rollCorrection);
    // void generateWalkingGait(float phase, Vector3f &V_exp, Vector3f &V_real, Direction direction);
    void calculateLegForces();
    void calculateInverseKinematics(float x, float y, float z, float &hipAngle, float &kneeAngle, bool isMirrored);
    void setServoAngle(int servoIndex, float angle);
    float clampAngle(float angle, float min, float max);
    float mirrorAngle(float angle, bool isMirrored);
    void validateMovement();
    void optimizeEnergyUsage();
};

#endif