#include <Arduino.h>
#include "QuadrupedRobot.h"

QuadrupedRobot robot;

// Gait parameters
const float TROT_CYCLE_TIME = 800.0f;   // ms原数值800
const float WALK_CYCLE_TIME = 1200.0f;  // ms原数值1200
const float GALLOP_CYCLE_TIME = 600.0f; // ms原数值600
const float BOUND_CYCLE_TIME = 500.0f;  // ms原数值500
// 声明全局变量
int testPhase = 0;
unsigned long lastTest = 0;
const unsigned long TEST_INTERVAL = 5000; // 每个测试间隔5秒
// Gait execution functions with mirroring support
void executeTrot(int cycles = 1, float velocity = 0.5f)
{
  robot.setGait(TROT);
  robot.setMaxSpeed(velocity);
  robot.setStepHeight(30.0f);
  robot.setCycleTime(TROT_CYCLE_TIME);

  // Diagonal pairs move together
  float phaseOffsets[] = {0.0f, 0.5f, 0.5f, 0.0f};
  for (int i = 0; i < 4; i++)
  {
    robot.setLegPhaseOffset(i, phaseOffsets[i]);
  }

  robot.walk(cycles, velocity);
}

void executeWalk(int cycles = 1, float velocity = 0.9f)
{
  robot.setGait(WALK);
  robot.setMaxSpeed(velocity);
  robot.setStepHeight(50.0f);
  robot.setCycleTime(WALK_CYCLE_TIME);

  // Sequential leg movement with proper left-right coordination
  float phaseOffsets[] = {0.0f, 0.25f, 0.5f, 0.75f};
  // float phaseOffsets[] = {0.0f, 0.5f, 0.5f, 0.0f};

  for (int i = 0; i < 4; i++)
  {
    robot.setLegPhaseOffset(i, phaseOffsets[i]);
  }

  robot.walk(cycles, velocity);
}

void executeGallop(int cycles = 1, float velocity = 0.7f)
{
  robot.setGait(GALLOP);
  robot.setMaxSpeed(velocity);
  robot.setStepHeight(45.0f);
  robot.setCycleTime(GALLOP_CYCLE_TIME);

  // Front legs move slightly apart, followed by back legs
  float phaseOffsets[] = {0.0f, 0.1f, 0.6f, 0.5f};
  for (int i = 0; i < 4; i++)
  {
    robot.setLegPhaseOffset(i, phaseOffsets[i]);
  }

  robot.walk(cycles, velocity);
}

void executeBound(int cycles = 1, float velocity = 0.6f)
{
  robot.setGait(BOUND);
  robot.setMaxSpeed(velocity);
  robot.setStepHeight(40.0f);
  robot.setCycleTime(BOUND_CYCLE_TIME);

  // Front pair and back pair move together
  float phaseOffsets[] = {0.0f, 0.0f, 0.5f, 0.5f};
  for (int i = 0; i < 4; i++)
  {
    robot.setLegPhaseOffset(i, phaseOffsets[i]);
  }

  robot.walk(cycles, velocity);
}

void executeTurn(float angle, Direction direction)
{
  robot.setGait(WALK);
  robot.setMaxSpeed(0.3f); // Slower speed for turning
  robot.setStepHeight(25.0f);

  if (direction == LEFT)
  {
    float phaseOffsets[] = {0.5f, 0.0f, 0.0f, 0.5f};
    for (int i = 0; i < 4; i++)
    {
      robot.setLegPhaseOffset(i, phaseOffsets[i]);
    }
  }
  else
  { // RIGHT
    float phaseOffsets[] = {0.0f, 0.5f, 0.5f, 0.0f};
    for (int i = 0; i < 4; i++)
    {
      robot.setLegPhaseOffset(i, phaseOffsets[i]);
    }
  }

  robot.turn(angle);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing quadruped robot...");
  // 设置所有电机的初始PWM值为300
  for (int i = 0; i < 8; i++)
  { // 假设四足机器人有8个电机
    robot.setMotorPWM(i, 300);
  }
  robot.init();
  robot.setMaxSpeed(0.5f);
  robot.setStepHeight(30.0f);
  robot.setBodyHeight(100.0f); // 原值是100
  robot.setPIDGains(0.8f, 0.2f, 0.1f);

  robot.calibrateSensors();
  delay(1000);

  Serial.println("Robot initialized and ready.");
  Serial.begin(115200);
  delay(1000); // 等待串口稳定

  // Serial.println("Initializing quadruped robot...");
  Serial.println("Quadruped Robot Debug Mode Starting...");

  // 启用调试模式
  robot.setDebugMode(true);

  // 初始化机器人
  Serial.println("Initializing robot...");
  robot.init();

  // 设置初始参数
  Serial.println("Setting initial parameters...");
  robot.setMaxSpeed(0.5f); // 设置较低的初始速度
  robot.setBodyHeight(80.0f);
  //   // // 验证MPU数据
  //   // const MpuData &initialData = robot.getSensorData();
  //   // Serial.println("\nInitial sensor readings:");
  //   // Serial.print("Pitch: ");
  //   // Serial.print(initialData.pitch);
  //   // Serial.print(" Roll: ");
  //   // Serial.println(initialData.roll);
  //   // Serial.println("Robot initialized, starting parameter test...");
  //   // 设置所有电机的初始PWM值为300
  for (int i = 0; i < 8; i++)
  { // 假设四足机器人有12个电机
    robot.setMotorPWM(i, 300);
  }
  delay(2000);
  robot.init();
  robot.setMaxSpeed(0.9f);
  robot.setStepHeight(30.0f);
  robot.setBodyHeight(100.0f); // 原值是100
  robot.setPIDGains(0.8f, 0.2f, 0.1f);

  robot.calibrateSensors();
  delay(1000);

  Serial.println("Robot initialized and ready.");
}

void loop()
{
  for (int i = 0; i < 8; i++)
  { // 假设四足机器人有12个电机
    robot.setMotorPWM(i, 300);
  }
  delay(2000);
  // robot.walk(6, 0.5, FORWARD);
  // Serial.println("Executing Walk");
  executeWalk(6);
  delay(2000);
  // executeTrot(6);
  // delay(2000);
  // static int currentGait = 0;
  // static unsigned long lastGaitChange = 0;
  // static const unsigned long GAIT_DURATION = 10000; // 5 seconds per gait
  // static const int DEMO_SEQUENCE_LENGTH = 5;

  // unsigned long currentTime = millis();

  // // Switch gaits every GAIT_DURATION milliseconds
  // if (currentTime - lastGaitChange >= GAIT_DURATION)
  // {
  //   robot.stand(); // Reset to standing position between gaits
  //   delay(500);    // Short pause between gaits

  //   switch (currentGait)
  //   {
  //   case 0:
  //     Serial.println("Executing Walk");
  //     executeWalk(6);
  //     break;
  //     // case 1:
  //     //   Serial.println("Executing Trot");
  //     //   executeTrot(6);
  //     //   break;
  //     // case 2:
  //     //   Serial.println("Executing Turn Left");
  //     //   executeTurn(90.0f, LEFT);
  //     //   break;
  //     // case 3:
  //     //   Serial.println("Executing Gallop");
  //     //   executeGallop(6);
  //     //   break;
  //     // case 4:
  //     //   Serial.println("Executing Bound");
  //     //   executeBound(6);
  //     //   break;
  //   }

  //   // currentGait = (currentGait + 1) % DEMO_SEQUENCE_LENGTH;
  //   // lastGaitChange = currentTime;
  // }

  // Main control loop
  robot.update();

  // // Safety checks and monitoring
  // robot.monitorHealth();

  // // Small delay to prevent CPU overload
  // delay(10);
  // =====================================================================================
  // 设置所有电机的初始PWM值为300
  // for (int i = 0; i < 8; i++)
  // { // 假设四足机器人有12个电机
  //   robot.setMotorPWM(i, 300);

  // }
  // delay(2000);
  // static int currentGait = 0;
  // static unsigned long lastGaitChange = 0;
  // static const unsigned long GAIT_DURATION = 5000; // 原值 5 seconds per gait
  // static const int DEMO_SEQUENCE_LENGTH = 5;

  // unsigned long currentTime = millis();

  // // Switch gaits every GAIT_DURATION milliseconds
  // if (currentTime - lastGaitChange >= GAIT_DURATION)
  // {
  //   robot.stand(); // Reset to standing position between gaits
  //   delay(500);    // Short pause between gaits

  //   switch (currentGait)
  //   {
  //   case 0:
  //     Serial.println("Executing Walk");
  //     executeWalk(5);
  //     break;
  //   case 1:
  //     Serial.println("Executing Trot");
  //     executeTrot(3);
  //     break;
  //   case 2:
  //     Serial.println("Executing Turn Left");
  //     executeTurn(90.0f, LEFT);
  //     break;
  //   case 3:
  //     Serial.println("Executing Gallop");
  //     executeGallop(3);
  //     break;
  //   case 4:
  //     Serial.println("Executing Bound");
  //     executeBound(3);
  //     break;
  //   }

  //   currentGait = (currentGait + 1) % DEMO_SEQUENCE_LENGTH;
  //   lastGaitChange = currentTime;
  // }

  // // Main control loop
  // robot.update();

  // // Safety checks and monitoring
  // robot.monitorHealth();

  // // Small delay to prevent CPU overload
  // delay(10);
  // // 持续更新机器人状态
  // robot.update();

  // // 读取并打印传感器数据 - 修改这部分
  // static unsigned long lastPrint = 0;
  // const unsigned long PRINT_INTERVAL = 5000; // 每500ms打印一次数据

  // if (millis() - lastPrint > PRINT_INTERVAL)
  // {
  //   lastPrint = millis();

  //   const MpuData &sensorData = robot.getSensorData();

  //   // // 无论是否移动都打印数据
  //   // Serial.println("\n--- Sensor Data ---");
  //   // Serial.print("Accel - X: ");
  //   // Serial.print(sensorData.accel_x);
  //   // Serial.print(" Y: ");
  //   // Serial.print(sensorData.accel_y);
  //   // Serial.print(" Z: ");
  //   // Serial.println(sensorData.accel_z);
  //   // delay(500);
  //   // Serial.print("Gyro - X: ");
  //   // Serial.print(sensorData.gyro_x);
  //   // Serial.print(" Y: ");
  //   // Serial.print(sensorData.gyro_y);
  //   // Serial.print(" Z: ");
  //   // Serial.println(sensorData.gyro_z);
  //  // Serial.print("Angles - Pitch: ");
  //  // Serial.print(sensorData.pitch);
  //   // Serial.print(" Roll: ");
  //   // Serial.println(sensorData.roll);
  //   // delay(500);
  //   // Serial.print("Robot State - Moving: ");
  //   // Serial.print(robot.isMoving());
  //   // Serial.print(" Balanced: ");
  //   // Serial.println(robot.isBalanced());
  //   // Serial.println("----------------");
  //   delay(500);
  // }

  // if (millis() - lastTest > TEST_INTERVAL)
  // {
  //   lastTest = millis();

  //   switch (testPhase)
  //   {
  //   case 0:
  //     Serial.println("Executing Walk");
  //     executeWalk(5);
  //     break;
  //     Serial.println("\nTesting walk with default parameters:");
  //     Serial.println("stepHeight: 30.0, strideLength: 80.0, cycleTime: 1000");
  //     // robot.walk(5, 0.3, FORWARD); // 低速测试
  //     executeWalk(5);
  //     break;
  //     delay(5000);
  //   case 1:
  //     Serial.println("\nTesting walk with increased step height:");
  //     robot.setStepHeight(40.0f); // 增加抬腿高度
  //     // robot.walk(5, 0.3, FORWARD);
  //     executeWalk(5);
  //     break;
  //     delay(5000);

  //   case 2:
  //     Serial.println("\nTesting walk with increased stride length:");
  //     robot.setStepHeight(30.0f);  // 恢复默认抬腿高度
  //     robot.setCycleTime(1200.0f); // 增加周期时间
  //     robot.walk(5, 0.3, FORWARD);
  //     break;
  //     delay(5000);

  //   case 3:
  //     Serial.println("\nTesting walk with increased speed:");
  //     robot.setCycleTime(1000.0f); // 恢复默认周期
  //     robot.walk(5, 0.5, FORWARD); // 增加速度
  //     break;
  //     delay(5000);

  //   case 4:
  //     Serial.println("\nTesting with custom parameters:");
  //     robot.setStepHeight(35.0f);
  //     robot.setCycleTime(1100.0f);
  //     robot.setBodyHeight(75.0f);
  //     robot.walk(5, 0.4, FORWARD);
  //     break;
  //     delay(5000);

  //   default:
  //     testPhase = -1; // 重置测试阶段
  //     Serial.println("\nTest cycle completed. Restarting...");
  //     break;
  //   }
  //   testPhase++;
  // }

  // 持续更新机器人状态
  // robot.update();
  // if (millis() - lastGaitChange > 5000) // 每5秒改变一次步态
  // {
  //   lastGaitChange = millis();
  //   currentGait = (currentGait + 1) % 4; // 循环切换步态

  //   switch (currentGait)
  //   {
  //   case 0:
  //     executeTrot(5);
  //     break;
  //   case 1:
  //     executeWalk(5);
  //     break;
  //   case 2:
  //     executeGallop(5);
  //     break;
  //   case 3:
  //     executeBound(5);
  //     break;
  //   }
  // }
  // // // 读取并打印传感器数据
  // // const MpuData &sensorData = robot.getSensorData();
  // // if (robot.isMoving())
  // // {
  // //   Serial.print("Pitch: ");
  // //   Serial.print(sensorData.pitch);
  // //   Serial.print(" Roll: ");
  // //   Serial.print(sensorData.roll);
  // //   Serial.print(" IsBalanced: ");
  // //   Serial.println(robot.isBalanced());
  // // }
  // delay(10); // 适当的循环延迟
}