// Extended Kalman filter estimating gravity direction and gyro bias from an
// IMU. Translated line for line from the Rust crate `dcmimu` 0.2.5
// (https://github.com/copterust/dcmimu, MIT license), whose generated
// equations are too long for Swift's type checker.
#ifndef DCM_IMU_H
#define DCM_IMU_H

typedef struct {
    float g0;
    float g0_2;
    float x0;
    float x1;
    float x2;
    float x3;
    float x4;
    float x5;
    float q_dcm2;
    float q_gyro_bias2;
    float r_acc2;
    float r_a2;
    float a0;
    float a1;
    float a2;
    float yaw;
    float pitch;
    float roll;
    float P00;
    float P01;
    float P02;
    float P03;
    float P04;
    float P05;
    float P10;
    float P11;
    float P12;
    float P13;
    float P14;
    float P15;
    float P20;
    float P21;
    float P22;
    float P23;
    float P24;
    float P25;
    float P30;
    float P31;
    float P32;
    float P33;
    float P34;
    float P35;
    float P40;
    float P41;
    float P42;
    float P43;
    float P44;
    float P45;
    float P50;
    float P51;
    float P52;
    float P53;
    float P54;
    float P55;
} DcmImu;

typedef struct {
    float yaw, pitch, roll;
} DcmAngles;

void dcm_imu_init(DcmImu *s);

/// `gx..gz` in rad/s, `ax..az` in m/s^2, `dt` in seconds.
DcmAngles dcm_imu_update(DcmImu *s, float gx_in, float gy_in, float gz_in,
                         float ax_in, float ay_in, float az_in, float dt);

#endif
