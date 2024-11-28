#include "MpuData.h"
#include <math.h>

void MpuData::calculateAngles()
{
    // Calculate pitch and roll angles from accelerometer data
    pitch_a = atan2(accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / M_PI;
    roll_a = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180.0 / M_PI;
    yaw_a = atan2(sqrt(accel_x * accel_x + accel_y * accel_y), accel_z) * 180.0 / M_PI;

    // Integrate gyroscope data
    static unsigned long lastUpdate = 0;
    unsigned long now = millis();
    float dt = (now - lastUpdate) / 1000.0f;
    lastUpdate = now;

    pitch_g += gyro_x * dt;
    roll_g += gyro_y * dt;
    yaw_g += gyro_z * dt;

    // Complementary filter to combine accelerometer and gyroscope data
    const float alpha = 0.96f;
    pitch = alpha * (pitch + gyro_x * dt) + (1.0f - alpha) * pitch_a;
    roll = alpha * (roll + gyro_y * dt) + (1.0f - alpha) * roll_a;
    yaw = yaw_g; // Yaw can only be accurately measured with a magnetometer

    // Calculate stable angles using low-pass filter
    const float beta = 0.1f;
    Sta_Pitch = Sta_Pitch * (1.0f - beta) + pitch * beta;
    Sta_Roll = Sta_Roll * (1.0f - beta) + roll * beta;
}

void MpuData::reset()
{
    accel_x = accel_y = accel_z = 0;
    gyro_x = gyro_y = gyro_z = 0;
    temp = 0;
    pitch_a = roll_a = yaw_a = 0;
    pitch_g = roll_g = yaw_g = 0;
    pitch = roll = yaw = 0;
    Sta_Pitch = Sta_Roll = 0;
}