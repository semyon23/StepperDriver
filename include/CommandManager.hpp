
#ifndef UNTITLED3_COMMANDMANAGER_HPP
#define UNTITLED3_COMMANDMANAGER_HPP

#include <Arduino.h>
#include <functional>


constexpr uint8_t MAX_NUMBER_DEVICE = 3;

using CommandCallback = std::function<void(const uint8_t* data, size_t len)>;

class CommandManager {
public:
    CommandManager();
    void executeCommand(uint8_t id_cmd, const uint8_t* data, size_t len);
    void addCommand(uint8_t id_cmd, CommandCallback cmd);
private:
    std::unordered_map<uint8_t, CommandCallback> commands;
};

namespace AXIS{
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

template <typename T>
    struct SerialFrame {
    uint8_t header{0xAA};
    uint8_t device_id{0};
    uint8_t cmd_id{0};
    uint8_t length{0};
    T data{};
    uint8_t crc8{0};
    uint8_t stopByte{0x55};
};


struct CommandData  {
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
    uint8_t transaction_id{0}; // 1
    float position[MAX_NUMBER_DEVICE]{0.0f, 0.0f, 0.0f}; // 12
    uint8_t stateId[MAX_NUMBER_DEVICE]{ 0,0, 0}; // 3
    uint8_t isHome[MAX_NUMBER_DEVICE]{ 0,0, 0}; // 3
    uint8_t isPowered[MAX_NUMBER_DEVICE]{ 0,0, 0}; // 3
    uint8_t isTarget[MAX_NUMBER_DEVICE]{0, 0, 0}; // 3
    uint8_t inMotion[MAX_NUMBER_DEVICE]{0, 0, 0}; // 3
    uint8_t errorID[MAX_NUMBER_DEVICE]{0, 0, 0}; // 3
};

struct FeedbackData {
    float position[MAX_NUMBER_DEVICE]{0.0f, 0.0f, 0.0f}; // 12
    float feedbackPos[MAX_NUMBER_DEVICE]{0.0f, 0.0f, 0.0f}; // 12
    float lagError[MAX_NUMBER_DEVICE]{0.0f, 0.0f, 0.0f}; // 12
    float velocity[MAX_NUMBER_DEVICE]{0.0f, 0.0f, 0.0f}; // 12
    float velocityFeedback[MAX_NUMBER_DEVICE]{0.0f, 0.0f, 0.0f}; // 12
    uint8_t inMotion[MAX_NUMBER_DEVICE]{ 0,0, 0}; // 3
    uint8_t inTarget[MAX_NUMBER_DEVICE]{0, 0, 0}; // 3
    uint8_t error[MAX_NUMBER_DEVICE]{0, 0, 0}; // 3
    uint8_t error_id[MAX_NUMBER_DEVICE]{0, 0, 0}; // 3
    uint8_t stateId[MAX_NUMBER_DEVICE]{0, 0, 0}; // 3
    uint8_t negativeLS[MAX_NUMBER_DEVICE]{0, 0, 0}; // 3
    uint8_t positiveLS[MAX_NUMBER_DEVICE]{0, 0, 0}; // 3
};


#pragma pack(pop)

constexpr uint8_t MAX_LENGH_COMMAND_BUFFER = 100;

class ProtocolHandler {
public:
    ProtocolHandler(CommandManager* cmdMgr, uint8_t ID);
    void processReadByte(uint8_t byte);
    void sendFrame(const uint8_t *data, size_t len);
    void setId(uint8_t id);
    uint8_t getId() const;
    template <typename T>
    SerialFrame<T> createFrame(uint8_t cmdId, const T& data) {
        SerialFrame<T> frame;
        frame.header    = 0xAA;
        frame.device_id = _id;
        frame.cmd_id    = cmdId;
        frame.length    = sizeof(T);
        frame.data      = data;
        frame.crc8      = this->calculateCRC(reinterpret_cast<uint8_t*>(&frame), sizeof(frame) - 2);
        frame.stopByte  = 0x55;

        return frame;
    };
private:
    void parseRequestFrame(uint8_t dataLen);
    uint8_t calculateCRC(const uint8_t *data, size_t len);
    static constexpr uint8_t HEADER = 0xAA;
    static constexpr uint8_t FOOTER = 0x55;
    static constexpr size_t MAX_LENGH_COMMAND_BUFFER = 256;
    static constexpr size_t FRAME_OVERHEAD = 6;
    uint8_t _id;
    CommandManager* _cmdMgr;
    size_t _bytesRead{0};
    std::array<uint8_t, MAX_LENGH_COMMAND_BUFFER> commandBuffer{};
};

#endif //UNTITLED3_COMMANDMANAGER_HPP
