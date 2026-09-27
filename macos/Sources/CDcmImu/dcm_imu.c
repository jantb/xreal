// Translated line for line from the Rust crate `dcmimu` 0.2.5
// (https://github.com/copterust/dcmimu), MIT license.
#include "dcm_imu.h"
#include <math.h>

void dcm_imu_init(DcmImu *s) {
    s->g0 = 9.81f;
    s->g0_2 = 9.81f * 9.81f;
    s->x0 = 0.0f;
    s->x1 = 0.0f;
    s->x2 = 1.0f;
    s->x3 = 0.0f;
    s->x4 = 0.0f;
    s->x5 = 0.0f;
    s->q_dcm2 = 0.1f * 0.1f;
    s->q_gyro_bias2 = 0.0001f * 0.0001f;
    s->r_acc2 = 0.5f * 0.5f;
    s->r_a2 = 10.0f * 10.0f;
    s->a0 = 0.0f;
    s->a1 = 0.0f;
    s->a2 = 0.0f;
    s->yaw = 0.0f;
    s->pitch = 0.0f;
    s->roll = 0.0f;
    s->P00 = 1.0f;
    s->P01 = 0.0f;
    s->P02 = 0.0f;
    s->P03 = 0.0f;
    s->P04 = 0.0f;
    s->P05 = 0.0f;
    s->P10 = 0.0f;
    s->P11 = 1.0f;
    s->P12 = 0.0f;
    s->P13 = 0.0f;
    s->P14 = 0.0f;
    s->P15 = 0.0f;
    s->P20 = 0.0f;
    s->P21 = 0.0f;
    s->P22 = 1.0f;
    s->P23 = 0.0f;
    s->P24 = 0.0f;
    s->P25 = 0.0f;
    s->P30 = 0.0f;
    s->P31 = 0.0f;
    s->P32 = 0.0f;
    s->P33 = (0.1f * 0.1f);
    s->P34 = 0.0f;
    s->P35 = 0.0f;
    s->P40 = 0.0f;
    s->P41 = 0.0f;
    s->P42 = 0.0f;
    s->P43 = 0.0f;
    s->P44 = (0.1f * 0.1f);
    s->P45 = 0.0f;
    s->P50 = 0.0f;
    s->P51 = 0.0f;
    s->P52 = 0.0f;
    s->P53 = 0.0f;
    s->P54 = 0.0f;
    s->P55 = (0.1f * 0.1f);
}

DcmAngles dcm_imu_update(DcmImu *s, float gx_in, float gy_in, float gz_in,
                         float ax_in, float ay_in, float az_in, float dt) {
    const float gx = gx_in;
        const float gy = gy_in;
        const float gz = gz_in;
        const float ax = ax_in;
        const float ay = ay_in;
        const float az = az_in;
        // save last state for rotation estimation
        const float x_last[3] = {s->x0, s->x1, s->x2};
        // state prediction
        const float x_0 =
            s->x0 - dt * (gy * s->x2 - gz * s->x1 + s->x1 * s->x5 - s->x2 * s->x4);
        const float x_1 =
            s->x1 + dt * (gx * s->x2 - gz * s->x0 + s->x0 * s->x5 - s->x2 * s->x3);
        const float x_2 =
            s->x2 - dt * (gx * s->x1 - gy * s->x0 + s->x0 * s->x4 - s->x1 * s->x3);
        const float x_3 = s->x3;
        const float x_4 = s->x4;
        const float x_5 = s->x5;
        // covariance prediction
        const float dt2 = dt * dt;
        const float P_00 = s->P00
            - dt * (s->P05 * s->x1 - s->P04 * s->x2 - s->P40 * s->x2
                + s->P50 * s->x1
                + s->P02 * (gy - s->x4)
                + s->P20 * (gy - s->x4)
                - s->P01 * (gz - s->x5)
                - s->P10 * (gz - s->x5))
            + dt2
                * (s->q_dcm2
                    - s->x1
                        * (s->P45 * s->x2 - s->P55 * s->x1 - s->P25 * (gy - s->x4)
                            + s->P15 * (gz - s->x5))
                    + s->x2
                        * (s->P44 * s->x2 - s->P54 * s->x1 - s->P24 * (gy - s->x4)
                            + s->P14 * (gz - s->x5))
                    - (gy - s->x4)
                        * (s->P42 * s->x2 - s->P52 * s->x1 - s->P22 * (gy - s->x4)
                            + s->P12 * (gz - s->x5))
                    + (gz - s->x5)
                        * (s->P41 * s->x2 - s->P51 * s->x1 - s->P21 * (gy - s->x4)
                            + s->P11 * (gz - s->x5)));
        const float P_01 = s->P01
            + dt * (s->P05 * s->x0 - s->P03 * s->x2 + s->P41 * s->x2
                - s->P51 * s->x1
                + s->P02 * (gx - s->x3)
                - s->P00 * (gz - s->x5)
                - s->P21 * (gy - s->x4)
                + s->P11 * (gz - s->x5))
            + dt2
                * (s->x0
                    * (s->P45 * s->x2 - s->P55 * s->x1 - s->P25 * (gy - s->x4)
                        + s->P15 * (gz - s->x5))
                    - s->x2
                        * (s->P43 * s->x2 - s->P53 * s->x1 - s->P23 * (gy - s->x4)
                            + s->P13 * (gz - s->x5))
                    + (gx - s->x3)
                        * (s->P42 * s->x2 - s->P52 * s->x1 - s->P22 * (gy - s->x4)
                            + s->P12 * (gz - s->x5))
                    - (gz - s->x5)
                        * (s->P40 * s->x2 - s->P50 * s->x1 - s->P20 * (gy - s->x4)
                            + s->P10 * (gz - s->x5)));
        const float P_02 = s->P02
            - dt * (s->P04 * s->x0 - s->P03 * s->x1 - s->P42 * s->x2
                + s->P52 * s->x1
                + s->P01 * (gx - s->x3)
                - s->P00 * (gy - s->x4)
                + s->P22 * (gy - s->x4)
                - s->P12 * (gz - s->x5))
            - dt2
                * (s->x0
                    * (s->P44 * s->x2 - s->P54 * s->x1 - s->P24 * (gy - s->x4)
                        + s->P14 * (gz - s->x5))
                    - s->x1
                        * (s->P43 * s->x2 - s->P53 * s->x1 - s->P23 * (gy - s->x4)
                            + s->P13 * (gz - s->x5))
                    + (gx - s->x3)
                        * (s->P41 * s->x2 - s->P51 * s->x1 - s->P21 * (gy - s->x4)
                            + s->P11 * (gz - s->x5))
                    - (gy - s->x4)
                        * (s->P40 * s->x2 - s->P50 * s->x1 - s->P20 * (gy - s->x4)
                            + s->P10 * (gz - s->x5)));
        const float P_03 = s->P03
            + dt * (s->P43 * s->x2 - s->P53 * s->x1 - s->P23 * (gy - s->x4)
                + s->P13 * (gz - s->x5));
        const float P_04 = s->P04
            + dt * (s->P44 * s->x2 - s->P54 * s->x1 - s->P24 * (gy - s->x4)
                + s->P14 * (gz - s->x5));
        const float P_05 = s->P05
            + dt * (s->P45 * s->x2 - s->P55 * s->x1 - s->P25 * (gy - s->x4)
                + s->P15 * (gz - s->x5));
        const float P_10 = s->P10
            - dt * (s->P15 * s->x1 - s->P14 * s->x2 + s->P30 * s->x2
                - s->P50 * s->x0
                - s->P20 * (gx - s->x3)
                + s->P12 * (gy - s->x4)
                + s->P00 * (gz - s->x5)
                - s->P11 * (gz - s->x5))
            + dt2
                * (s->x1
                    * (s->P35 * s->x2 - s->P55 * s->x0 - s->P25 * (gx - s->x3)
                        + s->P05 * (gz - s->x5))
                    - s->x2
                        * (s->P34 * s->x2 - s->P54 * s->x0 - s->P24 * (gx - s->x3)
                            + s->P04 * (gz - s->x5))
                    + (gy - s->x4)
                        * (s->P32 * s->x2 - s->P52 * s->x0 - s->P22 * (gx - s->x3)
                            + s->P02 * (gz - s->x5))
                    - (gz - s->x5)
                        * (s->P31 * s->x2 - s->P51 * s->x0 - s->P21 * (gx - s->x3)
                            + s->P01 * (gz - s->x5)));
        const float P_11 = s->P11
            + dt * (s->P15 * s->x0 - s->P13 * s->x2 - s->P31 * s->x2
                + s->P51 * s->x0
                + s->P12 * (gx - s->x3)
                + s->P21 * (gx - s->x3)
                - s->P01 * (gz - s->x5)
                - s->P10 * (gz - s->x5))
            + dt2
                * (s->q_dcm2
                    - s->x0
                        * (s->P35 * s->x2 - s->P55 * s->x0 - s->P25 * (gx - s->x3)
                            + s->P05 * (gz - s->x5))
                    + s->x2
                        * (s->P33 * s->x2 - s->P53 * s->x0 - s->P23 * (gx - s->x3)
                            + s->P03 * (gz - s->x5))
                    - (gx - s->x3)
                        * (s->P32 * s->x2 - s->P52 * s->x0 - s->P22 * (gx - s->x3)
                            + s->P02 * (gz - s->x5))
                    + (gz - s->x5)
                        * (s->P30 * s->x2 - s->P50 * s->x0 - s->P20 * (gx - s->x3)
                            + s->P00 * (gz - s->x5)));
        const float P_12 = s->P12
            - dt * (s->P14 * s->x0 - s->P13 * s->x1 + s->P32 * s->x2
                - s->P52 * s->x0
                + s->P11 * (gx - s->x3)
                - s->P22 * (gx - s->x3)
                - s->P10 * (gy - s->x4)
                + s->P02 * (gz - s->x5))
            + dt2
                * (s->x0
                    * (s->P34 * s->x2 - s->P54 * s->x0 - s->P24 * (gx - s->x3)
                        + s->P04 * (gz - s->x5))
                    - s->x1
                        * (s->P33 * s->x2 - s->P53 * s->x0 - s->P23 * (gx - s->x3)
                            + s->P03 * (gz - s->x5))
                    + (gx - s->x3)
                        * (s->P31 * s->x2 - s->P51 * s->x0 - s->P21 * (gx - s->x3)
                            + s->P01 * (gz - s->x5))
                    - (gy - s->x4)
                        * (s->P30 * s->x2 - s->P50 * s->x0 - s->P20 * (gx - s->x3)
                            + s->P00 * (gz - s->x5)));
        const float P_13 = s->P13
            - dt * (s->P33 * s->x2 - s->P53 * s->x0 - s->P23 * (gx - s->x3)
                + s->P03 * (gz - s->x5));
        const float P_14 = s->P14
            - dt * (s->P34 * s->x2 - s->P54 * s->x0 - s->P24 * (gx - s->x3)
                + s->P04 * (gz - s->x5));
        const float P_15 = s->P15
            - dt * (s->P35 * s->x2 - s->P55 * s->x0 - s->P25 * (gx - s->x3)
                + s->P05 * (gz - s->x5));
        const float P_20 = s->P20
            - dt * (s->P25 * s->x1 - s->P30 * s->x1 + s->P40 * s->x0
                - s->P24 * s->x2
                + s->P10 * (gx - s->x3)
                - s->P00 * (gy - s->x4)
                + s->P22 * (gy - s->x4)
                - s->P21 * (gz - s->x5))
            - dt2
                * (s->x1
                    * (s->P35 * s->x1 - s->P45 * s->x0 - s->P15 * (gx - s->x3)
                        + s->P05 * (gy - s->x4))
                    - s->x2
                        * (s->P34 * s->x1 - s->P44 * s->x0 - s->P14 * (gx - s->x3)
                            + s->P04 * (gy - s->x4))
                    + (gy - s->x4)
                        * (s->P32 * s->x1 - s->P42 * s->x0 - s->P12 * (gx - s->x3)
                            + s->P02 * (gy - s->x4))
                    - (gz - s->x5)
                        * (s->P31 * s->x1 - s->P41 * s->x0 - s->P11 * (gx - s->x3)
                            + s->P01 * (gy - s->x4)));
        const float P_21 = s->P21
            + dt * (s->P25 * s->x0 + s->P31 * s->x1
                - s->P41 * s->x0
                - s->P23 * s->x2
                - s->P11 * (gx - s->x3)
                + s->P01 * (gy - s->x4)
                + s->P22 * (gx - s->x3)
                - s->P20 * (gz - s->x5))
            + dt2
                * (s->x0
                    * (s->P35 * s->x1 - s->P45 * s->x0 - s->P15 * (gx - s->x3)
                        + s->P05 * (gy - s->x4))
                    - s->x2
                        * (s->P33 * s->x1 - s->P43 * s->x0 - s->P13 * (gx - s->x3)
                            + s->P03 * (gy - s->x4))
                    + (gx - s->x3)
                        * (s->P32 * s->x1 - s->P42 * s->x0 - s->P12 * (gx - s->x3)
                            + s->P02 * (gy - s->x4))
                    - (gz - s->x5)
                        * (s->P30 * s->x1 - s->P40 * s->x0 - s->P10 * (gx - s->x3)
                            + s->P00 * (gy - s->x4)));
        const float P_22 = s->P22
            - dt * (s->P24 * s->x0 - s->P23 * s->x1 - s->P32 * s->x1
                + s->P42 * s->x0
                + s->P12 * (gx - s->x3)
                + s->P21 * (gx - s->x3)
                - s->P02 * (gy - s->x4)
                - s->P20 * (gy - s->x4))
            + dt2
                * (s->q_dcm2
                    - s->x0
                        * (s->P34 * s->x1 - s->P44 * s->x0 - s->P14 * (gx - s->x3)
                            + s->P04 * (gy - s->x4))
                    + s->x1
                        * (s->P33 * s->x1 - s->P43 * s->x0 - s->P13 * (gx - s->x3)
                            + s->P03 * (gy - s->x4))
                    - (gx - s->x3)
                        * (s->P31 * s->x1 - s->P41 * s->x0 - s->P11 * (gx - s->x3)
                            + s->P01 * (gy - s->x4))
                    + (gy - s->x4)
                        * (s->P30 * s->x1 - s->P40 * s->x0 - s->P10 * (gx - s->x3)
                            + s->P00 * (gy - s->x4)));
        const float P_23 = s->P23
            + dt * (s->P33 * s->x1 - s->P43 * s->x0 - s->P13 * (gx - s->x3)
                + s->P03 * (gy - s->x4));
        const float P_24 = s->P24
            + dt * (s->P34 * s->x1 - s->P44 * s->x0 - s->P14 * (gx - s->x3)
                + s->P04 * (gy - s->x4));
        const float P_25 = s->P25
            + dt * (s->P35 * s->x1 - s->P45 * s->x0 - s->P15 * (gx - s->x3)
                + s->P05 * (gy - s->x4));
        const float P_30 = s->P30
            - dt * (s->P35 * s->x1 - s->P34 * s->x2 + s->P32 * (gy - s->x4)
                - s->P31 * (gz - s->x5));
        const float P_31 = s->P31
            + dt * (s->P35 * s->x0 - s->P33 * s->x2 + s->P32 * (gx - s->x3)
                - s->P30 * (gz - s->x5));
        const float P_32 = s->P32
            - dt * (s->P34 * s->x0 - s->P33 * s->x1 + s->P31 * (gx - s->x3)
                - s->P30 * (gy - s->x4));
        const float P_33 = s->P33 + dt2 * s->q_gyro_bias2;
        const float P_34 = s->P34;
        const float P_35 = s->P35;
        const float P_40 = s->P40
            - dt * (s->P45 * s->x1 - s->P44 * s->x2 + s->P42 * (gy - s->x4)
                - s->P41 * (gz - s->x5));
        const float P_41 = s->P41
            + dt * (s->P45 * s->x0 - s->P43 * s->x2 + s->P42 * (gx - s->x3)
                - s->P40 * (gz - s->x5));
        const float P_42 = s->P42
            - dt * (s->P44 * s->x0 - s->P43 * s->x1 + s->P41 * (gx - s->x3)
                - s->P40 * (gy - s->x4));
        const float P_43 = s->P43;
        const float P_44 = s->P44 + dt2 * s->q_gyro_bias2;
        const float P_45 = s->P45;
        const float P_50 = s->P50
            - dt * (s->P55 * s->x1 - s->P54 * s->x2 + s->P52 * (gy - s->x4)
                - s->P51 * (gz - s->x5));
        const float P_51 = s->P51
            + dt * (s->P55 * s->x0 - s->P53 * s->x2 + s->P52 * (gx - s->x3)
                - s->P50 * (gz - s->x5));
        const float P_52 = s->P52
            - dt * (s->P54 * s->x0 - s->P53 * s->x1 + s->P51 * (gx - s->x3)
                - s->P50 * (gy - s->x4));
        const float P_53 = s->P53;
        const float P_54 = s->P54;
        const float P_55 = s->P55 + dt2 * s->q_gyro_bias2;

        // Kalman innovation
        const float y0 = ax - s->g0 * x_0;
        const float y1 = ay - s->g0 * x_1;
        const float y2 = az - s->g0 * x_2;
        const float a_len = sqrtf(y0 * y0 + y1 * y1 + y2 * y2);

        const float S00 = s->r_acc2 + a_len * s->r_a2 + P_00 * s->g0_2;
        const float S01 = P_01 * s->g0_2;
        const float S02 = P_02 * s->g0_2;
        const float S10 = P_10 * s->g0_2;
        const float S11 = s->r_acc2 + a_len * s->r_a2 + P_11 * s->g0_2;
        const float S12 = P_12 * s->g0_2;
        const float S20 = P_20 * s->g0_2;
        const float S21 = P_21 * s->g0_2;
        const float S22 = s->r_acc2 + a_len * s->r_a2 + P_22 * s->g0_2;

        // Kalman gain
        const float invPart = 1.0f
            / (S00 * S11 * S22 - S00 * S12 * S21 - S01 * S10 * S22
                + S01 * S12 * S20
                + S02 * S10 * S21
                - S02 * S11 * S20);
        const float K00 = (s->g0
            * (P_02 * S10 * S21 - P_02 * S11 * S20 - P_01 * S10 * S22
                + P_01 * S12 * S20
                + P_00 * S11 * S22
                - P_00 * S12 * S21))
            * invPart;
        const float K01 = -(s->g0
            * (P_02 * S00 * S21 - P_02 * S01 * S20 - P_01 * S00 * S22
                + P_01 * S02 * S20
                + P_00 * S01 * S22
                - P_00 * S02 * S21))
            * invPart;
        const float K02 = (s->g0
            * (P_02 * S00 * S11 - P_02 * S01 * S10 - P_01 * S00 * S12
                + P_01 * S02 * S10
                + P_00 * S01 * S12
                - P_00 * S02 * S11))
            * invPart;
        const float K10 = (s->g0
            * (P_12 * S10 * S21 - P_12 * S11 * S20 - P_11 * S10 * S22
                + P_11 * S12 * S20
                + P_10 * S11 * S22
                - P_10 * S12 * S21))
            * invPart;
        const float K11 = -(s->g0
            * (P_12 * S00 * S21 - P_12 * S01 * S20 - P_11 * S00 * S22
                + P_11 * S02 * S20
                + P_10 * S01 * S22
                - P_10 * S02 * S21))
            * invPart;
        const float K12 = (s->g0
            * (P_12 * S00 * S11 - P_12 * S01 * S10 - P_11 * S00 * S12
                + P_11 * S02 * S10
                + P_10 * S01 * S12
                - P_10 * S02 * S11))
            * invPart;
        const float K20 = (s->g0
            * (P_22 * S10 * S21 - P_22 * S11 * S20 - P_21 * S10 * S22
                + P_21 * S12 * S20
                + P_20 * S11 * S22
                - P_20 * S12 * S21))
            * invPart;
        const float K21 = -(s->g0
            * (P_22 * S00 * S21 - P_22 * S01 * S20 - P_21 * S00 * S22
                + P_21 * S02 * S20
                + P_20 * S01 * S22
                - P_20 * S02 * S21))
            * invPart;
        const float K22 = (s->g0
            * (P_22 * S00 * S11 - P_22 * S01 * S10 - P_21 * S00 * S12
                + P_21 * S02 * S10
                + P_20 * S01 * S12
                - P_20 * S02 * S11))
            * invPart;
        const float K30 = (s->g0
            * (P_32 * S10 * S21 - P_32 * S11 * S20 - P_31 * S10 * S22
                + P_31 * S12 * S20
                + P_30 * S11 * S22
                - P_30 * S12 * S21))
            * invPart;
        const float K31 = -(s->g0
            * (P_32 * S00 * S21 - P_32 * S01 * S20 - P_31 * S00 * S22
                + P_31 * S02 * S20
                + P_30 * S01 * S22
                - P_30 * S02 * S21))
            * invPart;
        const float K32 = (s->g0
            * (P_32 * S00 * S11 - P_32 * S01 * S10 - P_31 * S00 * S12
                + P_31 * S02 * S10
                + P_30 * S01 * S12
                - P_30 * S02 * S11))
            * invPart;
        const float K40 = (s->g0
            * (P_42 * S10 * S21 - P_42 * S11 * S20 - P_41 * S10 * S22
                + P_41 * S12 * S20
                + P_40 * S11 * S22
                - P_40 * S12 * S21))
            * invPart;
        const float K41 = -(s->g0
            * (P_42 * S00 * S21 - P_42 * S01 * S20 - P_41 * S00 * S22
                + P_41 * S02 * S20
                + P_40 * S01 * S22
                - P_40 * S02 * S21))
            * invPart;
        const float K42 = (s->g0
            * (P_42 * S00 * S11 - P_42 * S01 * S10 - P_41 * S00 * S12
                + P_41 * S02 * S10
                + P_40 * S01 * S12
                - P_40 * S02 * S11))
            * invPart;
        const float K50 = (s->g0
            * (P_52 * S10 * S21 - P_52 * S11 * S20 - P_51 * S10 * S22
                + P_51 * S12 * S20
                + P_50 * S11 * S22
                - P_50 * S12 * S21))
            * invPart;
        const float K51 = -(s->g0
            * (P_52 * S00 * S21 - P_52 * S01 * S20 - P_51 * S00 * S22
                + P_51 * S02 * S20
                + P_50 * S01 * S22
                - P_50 * S02 * S21))
            * invPart;
        const float K52 = (s->g0
            * (P_52 * S00 * S11 - P_52 * S01 * S10 - P_51 * S00 * S12
                + P_51 * S02 * S10
                + P_50 * S01 * S12
                - P_50 * S02 * S11))
            * invPart;

        // update a posteriori
        s->x0 = x_0 + K00 * y0 + K01 * y1 + K02 * y2;
        s->x1 = x_1 + K10 * y0 + K11 * y1 + K12 * y2;
        s->x2 = x_2 + K20 * y0 + K21 * y1 + K22 * y2;
        s->x3 = x_3 + K30 * y0 + K31 * y1 + K32 * y2;
        s->x4 = x_4 + K40 * y0 + K41 * y1 + K42 * y2;
        s->x5 = x_5 + K50 * y0 + K51 * y1 + K52 * y2;

        // update a posteriori covariance
        const float r_adab = s->r_acc2 + a_len * s->r_a2;
        const float P__00 = P_00
            - s->g0 * (K00 * P_00 * 2.0f + K01 * P_01 + K01 * P_10 + K02 * P_02 + K02 * P_20)
            + (K00 * K00) * r_adab
            + (K01 * K01) * r_adab
            + (K02 * K02) * r_adab
            + s->g0_2
                * (K00 * (K00 * P_00 + K01 * P_10 + K02 * P_20)
                    + K01 * (K00 * P_01 + K01 * P_11 + K02 * P_21)
                    + K02 * (K00 * P_02 + K01 * P_12 + K02 * P_22));
        const float P__01 = P_01
            - s->g0
                * (K00 * P_01 + K01 * P_11 + K02 * P_21 + K10 * P_00 + K11 * P_01 + K12 * P_02)
            + s->g0_2
                * (K10 * (K00 * P_00 + K01 * P_10 + K02 * P_20)
                    + K11 * (K00 * P_01 + K01 * P_11 + K02 * P_21)
                    + K12 * (K00 * P_02 + K01 * P_12 + K02 * P_22))
            + K00 * K10 * r_adab
            + K01 * K11 * r_adab
            + K02 * K12 * r_adab;
        const float P__02 = P_02
            - s->g0
                * (K00 * P_02 + K01 * P_12 + K02 * P_22 + K20 * P_00 + K21 * P_01 + K22 * P_02)
            + s->g0_2
                * (K20 * (K00 * P_00 + K01 * P_10 + K02 * P_20)
                    + K21 * (K00 * P_01 + K01 * P_11 + K02 * P_21)
                    + K22 * (K00 * P_02 + K01 * P_12 + K02 * P_22))
            + K00 * K20 * r_adab
            + K01 * K21 * r_adab
            + K02 * K22 * r_adab;
        const float P__03 = P_03
            - s->g0
                * (K00 * P_03 + K01 * P_13 + K02 * P_23 + K30 * P_00 + K31 * P_01 + K32 * P_02)
            + s->g0_2
                * (K30 * (K00 * P_00 + K01 * P_10 + K02 * P_20)
                    + K31 * (K00 * P_01 + K01 * P_11 + K02 * P_21)
                    + K32 * (K00 * P_02 + K01 * P_12 + K02 * P_22))
            + K00 * K30 * r_adab
            + K01 * K31 * r_adab
            + K02 * K32 * r_adab;
        const float P__04 = P_04
            - s->g0
                * (K00 * P_04 + K01 * P_14 + K02 * P_24 + K40 * P_00 + K41 * P_01 + K42 * P_02)
            + s->g0_2
                * (K40 * (K00 * P_00 + K01 * P_10 + K02 * P_20)
                    + K41 * (K00 * P_01 + K01 * P_11 + K02 * P_21)
                    + K42 * (K00 * P_02 + K01 * P_12 + K02 * P_22))
            + K00 * K40 * r_adab
            + K01 * K41 * r_adab
            + K02 * K42 * r_adab;
        const float P__05 = P_05
            - s->g0
                * (K00 * P_05 + K01 * P_15 + K02 * P_25 + K50 * P_00 + K51 * P_01 + K52 * P_02)
            + s->g0_2
                * (K50 * (K00 * P_00 + K01 * P_10 + K02 * P_20)
                    + K51 * (K00 * P_01 + K01 * P_11 + K02 * P_21)
                    + K52 * (K00 * P_02 + K01 * P_12 + K02 * P_22))
            + K00 * K50 * r_adab
            + K01 * K51 * r_adab
            + K02 * K52 * r_adab;
        const float P__10 = P_10
            - s->g0
                * (K00 * P_10 + K01 * P_11 + K02 * P_12 + K10 * P_00 + K11 * P_10 + K12 * P_20)
            + s->g0_2
                * (K00 * (K10 * P_00 + K11 * P_10 + K12 * P_20)
                    + K01 * (K10 * P_01 + K11 * P_11 + K12 * P_21)
                    + K02 * (K10 * P_02 + K11 * P_12 + K12 * P_22))
            + K00 * K10 * r_adab
            + K01 * K11 * r_adab
            + K02 * K12 * r_adab;
        const float P__11 = P_11
            - s->g0 * (K10 * P_01 + K10 * P_10 + K11 * P_11 * 2.0f + K12 * P_12 + K12 * P_21)
            + (K10 * K10) * r_adab
            + (K11 * K11) * r_adab
            + (K12 * K12) * r_adab
            + s->g0_2
                * (K10 * (K10 * P_00 + K11 * P_10 + K12 * P_20)
                    + K11 * (K10 * P_01 + K11 * P_11 + K12 * P_21)
                    + K12 * (K10 * P_02 + K11 * P_12 + K12 * P_22));
        const float P__12 = P_12
            - s->g0
                * (K10 * P_02 + K11 * P_12 + K12 * P_22 + K20 * P_10 + K21 * P_11 + K22 * P_12)
            + s->g0_2
                * (K20 * (K10 * P_00 + K11 * P_10 + K12 * P_20)
                    + K21 * (K10 * P_01 + K11 * P_11 + K12 * P_21)
                    + K22 * (K10 * P_02 + K11 * P_12 + K12 * P_22))
            + K10 * K20 * r_adab
            + K11 * K21 * r_adab
            + K12 * K22 * r_adab;
        const float P__13 = P_13
            - s->g0
                * (K10 * P_03 + K11 * P_13 + K12 * P_23 + K30 * P_10 + K31 * P_11 + K32 * P_12)
            + s->g0_2
                * (K30 * (K10 * P_00 + K11 * P_10 + K12 * P_20)
                    + K31 * (K10 * P_01 + K11 * P_11 + K12 * P_21)
                    + K32 * (K10 * P_02 + K11 * P_12 + K12 * P_22))
            + K10 * K30 * r_adab
            + K11 * K31 * r_adab
            + K12 * K32 * r_adab;
        const float P__14 = P_14
            - s->g0
                * (K10 * P_04 + K11 * P_14 + K12 * P_24 + K40 * P_10 + K41 * P_11 + K42 * P_12)
            + s->g0_2
                * (K40 * (K10 * P_00 + K11 * P_10 + K12 * P_20)
                    + K41 * (K10 * P_01 + K11 * P_11 + K12 * P_21)
                    + K42 * (K10 * P_02 + K11 * P_12 + K12 * P_22))
            + K10 * K40 * r_adab
            + K11 * K41 * r_adab
            + K12 * K42 * r_adab;
        const float P__15 = P_15
            - s->g0
                * (K10 * P_05 + K11 * P_15 + K12 * P_25 + K50 * P_10 + K51 * P_11 + K52 * P_12)
            + s->g0_2
                * (K50 * (K10 * P_00 + K11 * P_10 + K12 * P_20)
                    + K51 * (K10 * P_01 + K11 * P_11 + K12 * P_21)
                    + K52 * (K10 * P_02 + K11 * P_12 + K12 * P_22))
            + K10 * K50 * r_adab
            + K11 * K51 * r_adab
            + K12 * K52 * r_adab;
        const float P__20 = P_20
            - s->g0
                * (K00 * P_20 + K01 * P_21 + K02 * P_22 + K20 * P_00 + K21 * P_10 + K22 * P_20)
            + s->g0_2
                * (K00 * (K20 * P_00 + K21 * P_10 + K22 * P_20)
                    + K01 * (K20 * P_01 + K21 * P_11 + K22 * P_21)
                    + K02 * (K20 * P_02 + K21 * P_12 + K22 * P_22))
            + K00 * K20 * r_adab
            + K01 * K21 * r_adab
            + K02 * K22 * r_adab;
        const float P__21 = P_21
            - s->g0
                * (K10 * P_20 + K11 * P_21 + K12 * P_22 + K20 * P_01 + K21 * P_11 + K22 * P_21)
            + s->g0_2
                * (K10 * (K20 * P_00 + K21 * P_10 + K22 * P_20)
                    + K11 * (K20 * P_01 + K21 * P_11 + K22 * P_21)
                    + K12 * (K20 * P_02 + K21 * P_12 + K22 * P_22))
            + K10 * K20 * r_adab
            + K11 * K21 * r_adab
            + K12 * K22 * r_adab;
        const float P__22 = P_22
            - s->g0 * (K20 * P_02 + K20 * P_20 + K21 * P_12 + K21 * P_21 + K22 * P_22 * 2.0f)
            + (K20 * K20) * r_adab
            + (K21 * K21) * r_adab
            + (K22 * K22) * r_adab
            + s->g0_2
                * (K20 * (K20 * P_00 + K21 * P_10 + K22 * P_20)
                    + K21 * (K20 * P_01 + K21 * P_11 + K22 * P_21)
                    + K22 * (K20 * P_02 + K21 * P_12 + K22 * P_22));
        const float P__23 = P_23
            - s->g0
                * (K20 * P_03 + K21 * P_13 + K22 * P_23 + K30 * P_20 + K31 * P_21 + K32 * P_22)
            + s->g0_2
                * (K30 * (K20 * P_00 + K21 * P_10 + K22 * P_20)
                    + K31 * (K20 * P_01 + K21 * P_11 + K22 * P_21)
                    + K32 * (K20 * P_02 + K21 * P_12 + K22 * P_22))
            + K20 * K30 * r_adab
            + K21 * K31 * r_adab
            + K22 * K32 * r_adab;
        const float P__24 = P_24
            - s->g0
                * (K20 * P_04 + K21 * P_14 + K22 * P_24 + K40 * P_20 + K41 * P_21 + K42 * P_22)
            + s->g0_2
                * (K40 * (K20 * P_00 + K21 * P_10 + K22 * P_20)
                    + K41 * (K20 * P_01 + K21 * P_11 + K22 * P_21)
                    + K42 * (K20 * P_02 + K21 * P_12 + K22 * P_22))
            + K20 * K40 * r_adab
            + K21 * K41 * r_adab
            + K22 * K42 * r_adab;
        const float P__25 = P_25
            - s->g0
                * (K20 * P_05 + K21 * P_15 + K22 * P_25 + K50 * P_20 + K51 * P_21 + K52 * P_22)
            + s->g0_2
                * (K50 * (K20 * P_00 + K21 * P_10 + K22 * P_20)
                    + K51 * (K20 * P_01 + K21 * P_11 + K22 * P_21)
                    + K52 * (K20 * P_02 + K21 * P_12 + K22 * P_22))
            + K20 * K50 * r_adab
            + K21 * K51 * r_adab
            + K22 * K52 * r_adab;
        const float P__30 = P_30
            - s->g0
                * (K00 * P_30 + K01 * P_31 + K02 * P_32 + K30 * P_00 + K31 * P_10 + K32 * P_20)
            + s->g0_2
                * (K00 * (K30 * P_00 + K31 * P_10 + K32 * P_20)
                    + K01 * (K30 * P_01 + K31 * P_11 + K32 * P_21)
                    + K02 * (K30 * P_02 + K31 * P_12 + K32 * P_22))
            + K00 * K30 * r_adab
            + K01 * K31 * r_adab
            + K02 * K32 * r_adab;
        const float P__31 = P_31
            - s->g0
                * (K10 * P_30 + K11 * P_31 + K12 * P_32 + K30 * P_01 + K31 * P_11 + K32 * P_21)
            + s->g0_2
                * (K10 * (K30 * P_00 + K31 * P_10 + K32 * P_20)
                    + K11 * (K30 * P_01 + K31 * P_11 + K32 * P_21)
                    + K12 * (K30 * P_02 + K31 * P_12 + K32 * P_22))
            + K10 * K30 * r_adab
            + K11 * K31 * r_adab
            + K12 * K32 * r_adab;
        const float P__32 = P_32
            - s->g0
                * (K20 * P_30 + K21 * P_31 + K22 * P_32 + K30 * P_02 + K31 * P_12 + K32 * P_22)
            + s->g0_2
                * (K20 * (K30 * P_00 + K31 * P_10 + K32 * P_20)
                    + K21 * (K30 * P_01 + K31 * P_11 + K32 * P_21)
                    + K22 * (K30 * P_02 + K31 * P_12 + K32 * P_22))
            + K20 * K30 * r_adab
            + K21 * K31 * r_adab
            + K22 * K32 * r_adab;
        const float P__33 = P_33
            - s->g0
                * (K30 * P_03 + K31 * P_13 + K30 * P_30 + K31 * P_31 + K32 * P_23 + K32 * P_32)
            + (K30 * K30) * r_adab
            + (K31 * K31) * r_adab
            + (K32 * K32) * r_adab
            + s->g0_2
                * (K30 * (K30 * P_00 + K31 * P_10 + K32 * P_20)
                    + K31 * (K30 * P_01 + K31 * P_11 + K32 * P_21)
                    + K32 * (K30 * P_02 + K31 * P_12 + K32 * P_22));
        const float P__34 = P_34
            - s->g0
                * (K30 * P_04 + K31 * P_14 + K32 * P_24 + K40 * P_30 + K41 * P_31 + K42 * P_32)
            + s->g0_2
                * (K40 * (K30 * P_00 + K31 * P_10 + K32 * P_20)
                    + K41 * (K30 * P_01 + K31 * P_11 + K32 * P_21)
                    + K42 * (K30 * P_02 + K31 * P_12 + K32 * P_22))
            + K30 * K40 * r_adab
            + K31 * K41 * r_adab
            + K32 * K42 * r_adab;
        const float P__35 = P_35
            - s->g0
                * (K30 * P_05 + K31 * P_15 + K32 * P_25 + K50 * P_30 + K51 * P_31 + K52 * P_32)
            + s->g0_2
                * (K50 * (K30 * P_00 + K31 * P_10 + K32 * P_20)
                    + K51 * (K30 * P_01 + K31 * P_11 + K32 * P_21)
                    + K52 * (K30 * P_02 + K31 * P_12 + K32 * P_22))
            + K30 * K50 * r_adab
            + K31 * K51 * r_adab
            + K32 * K52 * r_adab;
        const float P__40 = P_40
            - s->g0
                * (K00 * P_40 + K01 * P_41 + K02 * P_42 + K40 * P_00 + K41 * P_10 + K42 * P_20)
            + s->g0_2
                * (K00 * (K40 * P_00 + K41 * P_10 + K42 * P_20)
                    + K01 * (K40 * P_01 + K41 * P_11 + K42 * P_21)
                    + K02 * (K40 * P_02 + K41 * P_12 + K42 * P_22))
            + K00 * K40 * r_adab
            + K01 * K41 * r_adab
            + K02 * K42 * r_adab;
        const float P__41 = P_41
            - s->g0
                * (K10 * P_40 + K11 * P_41 + K12 * P_42 + K40 * P_01 + K41 * P_11 + K42 * P_21)
            + s->g0_2
                * (K10 * (K40 * P_00 + K41 * P_10 + K42 * P_20)
                    + K11 * (K40 * P_01 + K41 * P_11 + K42 * P_21)
                    + K12 * (K40 * P_02 + K41 * P_12 + K42 * P_22))
            + K10 * K40 * r_adab
            + K11 * K41 * r_adab
            + K12 * K42 * r_adab;
        const float P__42 = P_42
            - s->g0
                * (K20 * P_40 + K21 * P_41 + K22 * P_42 + K40 * P_02 + K41 * P_12 + K42 * P_22)
            + s->g0_2
                * (K20 * (K40 * P_00 + K41 * P_10 + K42 * P_20)
                    + K21 * (K40 * P_01 + K41 * P_11 + K42 * P_21)
                    + K22 * (K40 * P_02 + K41 * P_12 + K42 * P_22))
            + K20 * K40 * r_adab
            + K21 * K41 * r_adab
            + K22 * K42 * r_adab;
        const float P__43 = P_43
            - s->g0
                * (K30 * P_40 + K31 * P_41 + K32 * P_42 + K40 * P_03 + K41 * P_13 + K42 * P_23)
            + s->g0_2
                * (K30 * (K40 * P_00 + K41 * P_10 + K42 * P_20)
                    + K31 * (K40 * P_01 + K41 * P_11 + K42 * P_21)
                    + K32 * (K40 * P_02 + K41 * P_12 + K42 * P_22))
            + K30 * K40 * r_adab
            + K31 * K41 * r_adab
            + K32 * K42 * r_adab;
        const float P__44 = P_44
            - s->g0
                * (K40 * P_04 + K41 * P_14 + K40 * P_40 + K42 * P_24 + K41 * P_41 + K42 * P_42)
            + (K40 * K40) * r_adab
            + (K41 * K41) * r_adab
            + (K42 * K42) * r_adab
            + s->g0_2
                * (K40 * (K40 * P_00 + K41 * P_10 + K42 * P_20)
                    + K41 * (K40 * P_01 + K41 * P_11 + K42 * P_21)
                    + K42 * (K40 * P_02 + K41 * P_12 + K42 * P_22));
        const float P__45 = P_45
            - s->g0
                * (K40 * P_05 + K41 * P_15 + K42 * P_25 + K50 * P_40 + K51 * P_41 + K52 * P_42)
            + s->g0_2
                * (K50 * (K40 * P_00 + K41 * P_10 + K42 * P_20)
                    + K51 * (K40 * P_01 + K41 * P_11 + K42 * P_21)
                    + K52 * (K40 * P_02 + K41 * P_12 + K42 * P_22))
            + K40 * K50 * r_adab
            + K41 * K51 * r_adab
            + K42 * K52 * r_adab;
        const float P__50 = P_50
            - s->g0
                * (K00 * P_50 + K01 * P_51 + K02 * P_52 + K50 * P_00 + K51 * P_10 + K52 * P_20)
            + s->g0_2
                * (K00 * (K50 * P_00 + K51 * P_10 + K52 * P_20)
                    + K01 * (K50 * P_01 + K51 * P_11 + K52 * P_21)
                    + K02 * (K50 * P_02 + K51 * P_12 + K52 * P_22))
            + K00 * K50 * r_adab
            + K01 * K51 * r_adab
            + K02 * K52 * r_adab;
        const float P__51 = P_51
            - s->g0
                * (K10 * P_50 + K11 * P_51 + K12 * P_52 + K50 * P_01 + K51 * P_11 + K52 * P_21)
            + s->g0_2
                * (K10 * (K50 * P_00 + K51 * P_10 + K52 * P_20)
                    + K11 * (K50 * P_01 + K51 * P_11 + K52 * P_21)
                    + K12 * (K50 * P_02 + K51 * P_12 + K52 * P_22))
            + K10 * K50 * r_adab
            + K11 * K51 * r_adab
            + K12 * K52 * r_adab;
        const float P__52 = P_52
            - s->g0
                * (K20 * P_50 + K21 * P_51 + K22 * P_52 + K50 * P_02 + K51 * P_12 + K52 * P_22)
            + s->g0_2
                * (K20 * (K50 * P_00 + K51 * P_10 + K52 * P_20)
                    + K21 * (K50 * P_01 + K51 * P_11 + K52 * P_21)
                    + K22 * (K50 * P_02 + K51 * P_12 + K52 * P_22))
            + K20 * K50 * r_adab
            + K21 * K51 * r_adab
            + K22 * K52 * r_adab;
        const float P__53 = P_53
            - s->g0
                * (K30 * P_50 + K31 * P_51 + K32 * P_52 + K50 * P_03 + K51 * P_13 + K52 * P_23)
            + s->g0_2
                * (K30 * (K50 * P_00 + K51 * P_10 + K52 * P_20)
                    + K31 * (K50 * P_01 + K51 * P_11 + K52 * P_21)
                    + K32 * (K50 * P_02 + K51 * P_12 + K52 * P_22))
            + K30 * K50 * r_adab
            + K31 * K51 * r_adab
            + K32 * K52 * r_adab;
        const float P__54 = P_54
            - s->g0
                * (K40 * P_50 + K41 * P_51 + K42 * P_52 + K50 * P_04 + K51 * P_14 + K52 * P_24)
            + s->g0_2
                * (K40 * (K50 * P_00 + K51 * P_10 + K52 * P_20)
                    + K41 * (K50 * P_01 + K51 * P_11 + K52 * P_21)
                    + K42 * (K50 * P_02 + K51 * P_12 + K52 * P_22))
            + K40 * K50 * r_adab
            + K41 * K51 * r_adab
            + K42 * K52 * r_adab;
        const float P__55 = P_55
            - s->g0
                * (K50 * P_05 + K51 * P_15 + K52 * P_25 + K50 * P_50 + K51 * P_51 + K52 * P_52)
            + (K50 * K50) * r_adab
            + (K51 * K51) * r_adab
            + (K52 * K52) * r_adab
            + s->g0_2
                * (K50 * (K50 * P_00 + K51 * P_10 + K52 * P_20)
                    + K51 * (K50 * P_01 + K51 * P_11 + K52 * P_21)
                    + K52 * (K50 * P_02 + K51 * P_12 + K52 * P_22));

        const float len = sqrtf(s->x0 * s->x0 + s->x1 * s->x1 + s->x2 * s->x2);
        const float invlen3 = 1.0f / (len * len * len);
        const float invlen32 = invlen3 * invlen3;

        const float x1_x2 = s->x1 * s->x1 + s->x2 * s->x2;
        const float x0_x2 = s->x0 * s->x0 + s->x2 * s->x2;
        const float x0_x1 = s->x0 * s->x0 + s->x1 * s->x1;

        // normalized a posteriori covariance
        s->P00 = invlen32
            * (-x1_x2 * (-P__00 * x1_x2 + P__10 * s->x0 * s->x1 + P__20 * s->x0 * s->x2)
                + s->x0
                    * s->x1
                    * (-P__01 * x1_x2 + P__11 * s->x0 * s->x1 + P__21 * s->x0 * s->x2)
                + s->x0
                    * s->x2
                    * (-P__02 * x1_x2 + P__12 * s->x0 * s->x1 + P__22 * s->x0 * s->x2));
        s->P01 = invlen32
            * (-x0_x2 * (-P__01 * x1_x2 + P__11 * s->x0 * s->x1 + P__21 * s->x0 * s->x2)
                + s->x0
                    * s->x1
                    * (-P__00 * x1_x2 + P__10 * s->x0 * s->x1 + P__20 * s->x0 * s->x2)
                + s->x1
                    * s->x2
                    * (-P__02 * x1_x2 + P__12 * s->x0 * s->x1 + P__22 * s->x0 * s->x2));
        s->P02 = invlen32
            * (-x0_x1 * (-P__02 * x1_x2 + P__12 * s->x0 * s->x1 + P__22 * s->x0 * s->x2)
                + s->x0
                    * s->x2
                    * (-P__00 * x1_x2 + P__10 * s->x0 * s->x1 + P__20 * s->x0 * s->x2)
                + s->x1
                    * s->x2
                    * (-P__01 * x1_x2 + P__11 * s->x0 * s->x1 + P__21 * s->x0 * s->x2));
        s->P03 =
            -invlen3 * (-P__03 * x1_x2 + P__13 * s->x0 * s->x1 + P__23 * s->x0 * s->x2);
        s->P04 =
            -invlen3 * (-P__04 * x1_x2 + P__14 * s->x0 * s->x1 + P__24 * s->x0 * s->x2);
        s->P05 =
            -invlen3 * (-P__05 * x1_x2 + P__15 * s->x0 * s->x1 + P__25 * s->x0 * s->x2);
        s->P10 = invlen32
            * (-x1_x2 * (-P__10 * x0_x2 + P__00 * s->x0 * s->x1 + P__20 * s->x1 * s->x2)
                + s->x0
                    * s->x1
                    * (-P__11 * x0_x2 + P__01 * s->x0 * s->x1 + P__21 * s->x1 * s->x2)
                + s->x0
                    * s->x2
                    * (-P__12 * x0_x2 + P__02 * s->x0 * s->x1 + P__22 * s->x1 * s->x2));
        s->P11 = invlen32
            * (-x0_x2 * (-P__11 * x0_x2 + P__01 * s->x0 * s->x1 + P__21 * s->x1 * s->x2)
                + s->x0
                    * s->x1
                    * (-P__10 * x0_x2 + P__00 * s->x0 * s->x1 + P__20 * s->x1 * s->x2)
                + s->x1
                    * s->x2
                    * (-P__12 * x0_x2 + P__02 * s->x0 * s->x1 + P__22 * s->x1 * s->x2));
        s->P12 = invlen32
            * (-x0_x1 * (-P__12 * x0_x2 + P__02 * s->x0 * s->x1 + P__22 * s->x1 * s->x2)
                + s->x0
                    * s->x2
                    * (-P__10 * x0_x2 + P__00 * s->x0 * s->x1 + P__20 * s->x1 * s->x2)
                + s->x1
                    * s->x2
                    * (-P__11 * x0_x2 + P__01 * s->x0 * s->x1 + P__21 * s->x1 * s->x2));
        s->P13 =
            -invlen3 * (-P__13 * x0_x2 + P__03 * s->x0 * s->x1 + P__23 * s->x1 * s->x2);
        s->P14 =
            -invlen3 * (-P__14 * x0_x2 + P__04 * s->x0 * s->x1 + P__24 * s->x1 * s->x2);
        s->P15 =
            -invlen3 * (-P__15 * x0_x2 + P__05 * s->x0 * s->x1 + P__25 * s->x1 * s->x2);
        s->P20 = invlen32
            * (-x1_x2 * (-P__20 * x0_x1 + P__00 * s->x0 * s->x2 + P__10 * s->x1 * s->x2)
                + s->x0
                    * s->x1
                    * (-P__21 * x0_x1 + P__01 * s->x0 * s->x2 + P__11 * s->x1 * s->x2)
                + s->x0
                    * s->x2
                    * (-P__22 * x0_x1 + P__02 * s->x0 * s->x2 + P__12 * s->x1 * s->x2));
        s->P21 = invlen32
            * (-x0_x2 * (-P__21 * x0_x1 + P__01 * s->x0 * s->x2 + P__11 * s->x1 * s->x2)
                + s->x0
                    * s->x1
                    * (-P__20 * x0_x1 + P__00 * s->x0 * s->x2 + P__10 * s->x1 * s->x2)
                + s->x1
                    * s->x2
                    * (-P__22 * x0_x1 + P__02 * s->x0 * s->x2 + P__12 * s->x1 * s->x2));
        s->P22 = invlen32
            * (-x0_x1 * (-P__22 * x0_x1 + P__02 * s->x0 * s->x2 + P__12 * s->x1 * s->x2)
                + s->x0
                    * s->x2
                    * (-P__20 * x0_x1 + P__00 * s->x0 * s->x2 + P__10 * s->x1 * s->x2)
                + s->x1
                    * s->x2
                    * (-P__21 * x0_x1 + P__01 * s->x0 * s->x2 + P__11 * s->x1 * s->x2));
        s->P23 =
            -invlen3 * (-P__23 * x0_x1 + P__03 * s->x0 * s->x2 + P__13 * s->x1 * s->x2);
        s->P24 =
            -invlen3 * (-P__24 * x0_x1 + P__04 * s->x0 * s->x2 + P__14 * s->x1 * s->x2);
        s->P25 =
            -invlen3 * (-P__25 * x0_x1 + P__05 * s->x0 * s->x2 + P__15 * s->x1 * s->x2);
        s->P30 =
            -invlen3 * (-P__30 * x1_x2 + P__31 * s->x0 * s->x1 + P__32 * s->x0 * s->x2);
        s->P31 =
            -invlen3 * (-P__31 * x0_x2 + P__30 * s->x0 * s->x1 + P__32 * s->x1 * s->x2);
        s->P32 =
            -invlen3 * (-P__32 * x0_x1 + P__30 * s->x0 * s->x2 + P__31 * s->x1 * s->x2);
        s->P33 = P__33;
        s->P34 = P__34;
        s->P35 = P__35;
        s->P40 =
            -invlen3 * (-P__40 * x1_x2 + P__41 * s->x0 * s->x1 + P__42 * s->x0 * s->x2);
        s->P41 =
            -invlen3 * (-P__41 * x0_x2 + P__40 * s->x0 * s->x1 + P__42 * s->x1 * s->x2);
        s->P42 =
            -invlen3 * (-P__42 * x0_x1 + P__40 * s->x0 * s->x2 + P__41 * s->x1 * s->x2);
        s->P43 = P__43;
        s->P44 = P__44;
        s->P45 = P__45;
        s->P50 =
            -invlen3 * (-P__50 * x1_x2 + P__51 * s->x0 * s->x1 + P__52 * s->x0 * s->x2);
        s->P51 =
            -invlen3 * (-P__51 * x0_x2 + P__50 * s->x0 * s->x1 + P__52 * s->x1 * s->x2);
        s->P52 =
            -invlen3 * (-P__52 * x0_x1 + P__50 * s->x0 * s->x2 + P__51 * s->x1 * s->x2);
        s->P53 = P__53;
        s->P54 = P__54;
        s->P55 = P__55;
        // normalized a posteriori state
        s->x0 = s->x0 / len;
        s->x1 = s->x1 / len;
        s->x2 = s->x2 / len;
        // compute Euler angles
        const float u_nb1 = gy - s->x4;
        const float u_nb2 = gz - s->x5;
        const float cy = cosf(s->yaw);
        const float sy = sinf(s->yaw);
        const float d = sqrtf(x_last[1] * x_last[1] + x_last[2] * x_last[2]);
        const float d_inv = 1.0f / d;
        // compute needed parts of rotation matrix R (state and angle based version, equivalent with the commented version above)
        const float R11 = cy * d;
        const float R12 = -(x_last[2] * sy + x_last[0] * x_last[1] * cy) * d_inv;
        const float R13 = (x_last[1] * sy - x_last[0] * x_last[2] * cy) * d_inv;
        const float R21 = sy * d;
        const float R22 = (x_last[2] * cy - x_last[0] * x_last[1] * sy) * d_inv;
        const float R23 = -(x_last[1] * cy + x_last[0] * x_last[2] * sy) * d_inv;

        // update needed parts of R for yaw computation
        const float R11_new = R11 + dt * (u_nb2 * R12 - u_nb1 * R13);
        const float R21_new = R21 + dt * (u_nb2 * R22 - u_nb1 * R23);

        s->yaw = atan2f(R21_new, R11_new);
        s->pitch = asinf(-s->x0);
        s->roll = atan2f(s->x1, s->x2);

        // save the estimated non-gravitational acceleration
        s->a0 = ax - s->x0 * s->g0;
        s->a1 = ay - s->x1 * s->g0;
        s->a2 = az - s->x2 * s->g0;
    DcmAngles angles = {s->yaw, s->pitch, s->roll};
    return angles;
}
