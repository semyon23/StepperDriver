
#ifndef UNTITLED3_COMMANDMANAGER_HPP
#define UNTITLED3_COMMANDMANAGER_HPP

#include <Arduino.h>
#include <functional>
#include "protocol_types.h"

using CommandCallback = std::function<void(const uint8_t* data, size_t len)>;

class CommandManager {
public:
    CommandManager();
    void executeCommand(uint8_t id_cmd, const uint8_t* data, size_t len);
    void addCommand(uint8_t id_cmd, CommandCallback cmd);
private:
    std::unordered_map<uint8_t, CommandCallback> commands;
};

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
