#ifndef MPU_DATA_H
#define MPU_DATA_H

#include <Arduino.h>

struct MpuData
{
    float accel_x, accel_y, accel_z; // Accelerometer data
    float gyro_x, gyro_y, gyro_z;    // Gyroscope data
    float temp;                      // Temperature data
    float pitch_a, roll_a, yaw_a;    // Attitude angles based on accelerometer
    float pitch_g, roll_g, yaw_g;    // Attitude angles based on gyroscope
    float pitch, roll, yaw;          // Fused attitude angles
    float Sta_Pitch;                 // Stable pitch angle
    float Sta_Roll;                  // Stable roll angle

    MpuData() : accel_x(0), accel_y(0), accel_z(0),
                gyro_x(0), gyro_y(0), gyro_z(0),
                temp(0),
                pitch_a(0), roll_a(0), yaw_a(0),
                pitch_g(0), roll_g(0), yaw_g(0),
                pitch(0), roll(0), yaw(0),
                Sta_Pitch(0), Sta_Roll(0) {}

    // Calculate attitude angles using sensor fusion
    void calculateAngles();

    // Reset all values to zero
    void reset();
};

#endif // MPU_DATA_H