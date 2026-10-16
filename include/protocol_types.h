#ifndef STEPPER_DRIVER_PROTOCOL_TYPES_H
#define STEPPER_DRIVER_PROTOCOL_TYPES_H

#pragma once

#include <Arduino.h>

constexpr uint8_t MAX_NUMBER_DEVICE = 3;

namespace AXIS {
    enum ID : uint8_t {
        X           = 0,
        Y           = 1,
        C           = 2,
        GROUP       = 3,
    };
}

namespace COMMAND {
    enum ID : uint8_t {
        NO_COMMAND         = 0,
        POWER_ENABLE       = 1,
        MOVE_RELATIVE      = 2,
        MOVE_ABSOLUTE      = 3,
        MOVE_CONTINUE      = 4,
        MOVE_HOME          = 5,
        HALT               = 6,
        EMERGENCY_STOP     = 7,
        SET_POSITION       = 8,
        GET_STATUS         = 10,
        PING               = 11,
        GET_FEEDBACK       = 20,
        RESET_ERROR        = 30,
        CHANGE_ID_DEVICE   = 250,
        SET_LIMITS         = 251,
        SET_HOMING         = 252,
        JUMP_TO_BOOTLOADER = 255,
    };
}

#pragma pack(push, 1)

template<typename T>
struct SerialFrame {
    uint8_t header{0xAA};
    uint8_t device_id{0};
    uint8_t cmd_id{0};
    uint8_t length{0};
    T data{};
    uint8_t crc8{0};
    uint8_t stopByte{0x55};
};

struct CommandData {
    uint8_t transaction_id{0};
    uint8_t axis_id{0};
    uint8_t power_enable{0};
    float position[MAX_NUMBER_DEVICE]{0.0f, 0.0f, 0.0f};
    float velocity{0};
    float acceleration{0};
    float deceleration{0};
    int8_t direction[MAX_NUMBER_DEVICE]{0, 0, 0};
};

struct StatusData {
    uint8_t transaction_id{0};
    float position[MAX_NUMBER_DEVICE]{0.0f, 0.0f, 0.0f};
    uint8_t stateId[MAX_NUMBER_DEVICE]{0, 0, 0};
    uint8_t isHome[MAX_NUMBER_DEVICE]{0, 0, 0};
    uint8_t isPowered[MAX_NUMBER_DEVICE]{0, 0, 0};
    uint8_t isTarget[MAX_NUMBER_DEVICE]{0, 0, 0};
    uint8_t inMotion[MAX_NUMBER_DEVICE]{0, 0, 0};
    uint8_t errorID[MAX_NUMBER_DEVICE]{0, 0, 0};
};

struct FeedbackData {
    float position[MAX_NUMBER_DEVICE]{0.0f, 0.0f, 0.0f};
    float feedbackPos[MAX_NUMBER_DEVICE]{0.0f, 0.0f, 0.0f};
    float lagError[MAX_NUMBER_DEVICE]{0.0f, 0.0f, 0.0f};
    float velocity[MAX_NUMBER_DEVICE]{0.0f, 0.0f, 0.0f};
    float velocityFeedback[MAX_NUMBER_DEVICE]{0.0f, 0.0f, 0.0f};
    uint8_t inMotion[MAX_NUMBER_DEVICE]{0, 0, 0};
    uint8_t inTarget[MAX_NUMBER_DEVICE]{0, 0, 0};
    uint8_t error[MAX_NUMBER_DEVICE]{0, 0, 0};
    uint8_t error_id[MAX_NUMBER_DEVICE]{0, 0, 0};
    uint8_t stateId[MAX_NUMBER_DEVICE]{0, 0, 0};
    uint8_t negativeLS[MAX_NUMBER_DEVICE]{0, 0, 0};
    uint8_t positiveLS[MAX_NUMBER_DEVICE]{0, 0, 0};
};

#pragma pack(pop)

#endif //STEPPER_DRIVER_PROTOCOL_TYPES_H
