//
// Created by ahmed on 4/20/2023.
//

#ifndef ESP_HOVER_MODEL_H
#define ESP_HOVER_MODEL_H

#include <Arduino.h>

struct PIDModel {
    double proportional{0};
    double integral{0};
    double derivative{0};
    double system_constrained_total{0};
};

struct ThrustModel {
    int motor_a_thrust{0};
    int motor_b_thrust{0};
    int motor_c_thrust{0};
    int motor_d_thrust{0};
    int motor_e_thrust{0};
    int motor_f_thrust{0};
};

enum class ConnectionState {
    CONNECTED, DISCONNECTED
};

struct __attribute__((__packed__)) FlightRecord
{
    uint32_t millis; // 4 bytes

    uint8_t pitchSetPoint; // 3 bytes
    uint8_t rollSetPoint;
    uint8_t yawSetPoint;

    int16_t altitude; // 2 bytes

    int16_t pitch; // 6 bytes
    int16_t roll;
    int16_t yawRate;

    int16_t pitch_p; // 6 bytes
    int16_t pitch_i;
    int16_t pitch_d;

    int16_t roll_p; // 6 bytes
    int16_t roll_i;
    int16_t roll_d;

    int16_t yaw_p; // 6 bytes
    int16_t yaw_i;
    int16_t yaw_d;

    uint8_t motor_a_thrust; // 4 bytes
    uint8_t motor_b_thrust;
    uint8_t motor_c_thrust;
    uint8_t motor_d_thrust;
};

struct __attribute__((__packed__)) AttitudeModel{
    int16_t thrust;
    int16_t pitchSetPoint;
    int16_t rollSetPoint;
    int16_t yawSetPoint;
};

struct __attribute__((__packed__)) GainModel{
    float pitch_kp;
    float pitch_ki;
    float pitch_kd;

    float roll_kp;
    float roll_ki;
    float roll_kd;

    float yaw_kp;
    float yaw_ki;
    float yaw_kd;
};

#endif //ESP_HOVER_MODEL_H