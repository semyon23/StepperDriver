#include "../include/CommandManager.hpp"

CommandManager::CommandManager() {}

void CommandManager::executeCommand(uint8_t id_cmd, const uint8_t* data, size_t len) {
    auto it = commands.find(id_cmd);
    if (it != commands.end()) {
        it->second(data, len);
    } else {
        Serial.print("Unknown command: ");
        Serial.println(id_cmd);
    }
}

void CommandManager::addCommand(uint8_t id_cmd, CommandCallback cmd) {
    commands[id_cmd] = std::move(cmd);
}

ProtocolHandler::ProtocolHandler(CommandManager* cmdMgr, uint8_t id)
    : _cmdMgr(cmdMgr), _id(id){
    _bytesRead = 0;
    pinMode(PB5, OUTPUT);
    digitalWrite(PB5, LOW);
}

void ProtocolHandler::setId(uint8_t id) {
    _id = id;
}

uint8_t ProtocolHandler::getId() const {
    return _id;
}

void ProtocolHandler::processReadByte(uint8_t byte) {
    if (_bytesRead == 0 && byte != HEADER) return;

    if (_bytesRead == 1 && byte != _id) {
        _bytesRead = (byte == HEADER) ? 1 : 0;
        return;
    }

    if (_bytesRead == 3 && byte > (MAX_LENGH_COMMAND_BUFFER - FRAME_OVERHEAD)) {
        _bytesRead = 0;
        return;
    }
    commandBuffer[_bytesRead++] = byte;
    if (_bytesRead > 3) {
        uint8_t dataLen = commandBuffer[3];
        size_t totalFrameSize = FRAME_OVERHEAD + dataLen;

        if (_bytesRead == totalFrameSize) {
            if (byte == FOOTER) {
                this->parseRequestFrame(dataLen);
            }
            _bytesRead = 0;
        }
    }
}

void ProtocolHandler::parseRequestFrame(uint8_t dataLen) {
    size_t fullSize = 6 + dataLen;

    uint8_t* dataPtr = commandBuffer.data() + 4;
    uint8_t cmdId = commandBuffer[2];
    uint8_t crc = commandBuffer[4 + dataLen];

    if (crc != this->calculateCRC(commandBuffer.data(), 4 + dataLen)) {
        return;
    }

    _cmdMgr->executeCommand(cmdId, dataPtr, dataLen);
}

uint8_t ProtocolHandler::calculateCRC(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    return crc;
}

void ProtocolHandler::sendFrame(const uint8_t *data, size_t len) {
    digitalWrite(PB5, HIGH);
    Serial1.write(data, len);
    Serial1.flush();
    delayMicroseconds(50);
    digitalWrite(PB5, LOW);
}



