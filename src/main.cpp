#include "../include/AxesManager.hpp"
#include "../include/AxisHardware.hpp"
#include <EEPROM.h>
//sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D "/home/ktc/pio_project/firmware.bin"
//sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D "/home/ktc/pio_project/firmware.bin"
uint8_t DEFAULT_ID = 1;

constexpr int32_t TICK_RATE_PULSE = 50000;
constexpr float TICK_RATE = 0.00002f;

constexpr int32_t TICK_RATE_PROFILE = 2000;
constexpr float CYCLE_TIME = 0.0005f;

const AxisConfig config[MAX_NUMBER_DEVICE] = {
    {
        .step = {GPIOA, GPIO_PIN_7},
        .dir  = {GPIOA, GPIO_PIN_6},
        .enable = {GPIOB, GPIO_PIN_12},
        .chipSelect = {GPIOB, GPIO_PIN_8},
        .fault = {nullptr, 0},
        .pulseHome = {nullptr, 0}
    },
    {
        .step = {GPIOB, GPIO_PIN_10},
        .dir  = {GPIOB, GPIO_PIN_2},
        .enable = {nullptr, 0},
        .chipSelect = {GPIOB, GPIO_PIN_9},
        .fault = {nullptr, 0},
        .pulseHome = {nullptr, 0}
    },
    {
        .step = {GPIOA, GPIO_PIN_8},
        .dir  = {GPIOA, GPIO_PIN_15},
        .enable = {nullptr, 0},
        .chipSelect = {nullptr, 0},
        .fault = {nullptr, 0},
        .pulseHome = {nullptr, 0}
    }
};

AxisLimitSwitch limitsSwitch[MAX_NUMBER_DEVICE] = {
    {.negative = false, .positive = false},
    {.negative = false, .positive = false},
    {.negative = false, .positive = false}
};

AxisHardware hwX(config[0]);
AxisHardware hwY(config[1]);
AxisHardware hwC(config[2]);

// EncoderMT6835 MT6835;
// EncoderHandler encoderAxisX(&MT6835, &hwX, 1600, true);
// EncoderHandler encoderAxisY(&MT6835, &hwY, 1600, true);

AxisControl axisX(1, &hwX, &limitsSwitch[0], CYCLE_TIME, TICK_RATE, true);
AxisControl axisY(2, &hwY, &limitsSwitch[1], CYCLE_TIME, TICK_RATE, true);
AxisControl axisC(3, &hwC, &limitsSwitch[2], CYCLE_TIME, TICK_RATE, false);


CommandManager CmdManager;

ProtocolHandler protocolHandler(&CmdManager, DEFAULT_ID);

HardwareTimer timer_profile(TIM2);
HardwareTimer timer_pulse(TIM3);

AxesManager Controller(&axisX, &axisY, &axisC);

volatile bool readyToReadEncoder = false;

void handleProfileInterrupt() {
    axisX.update();
    axisY.update();
    axisC.update();
    readyToReadEncoder = true;
}

void handlePulseInterrupt() {
    axisX.getPulseDriver()->run();
    axisY.getPulseDriver()->run();
    axisC.getPulseDriver()->run();
}

void handleCompareInterrupt() {
    axisX.getPulseDriver()->handleStepLow();
    axisY.getPulseDriver()->handleStepLow();
    axisC.getPulseDriver()->handleStepLow();
}

constexpr int pin_negX = PA0;
constexpr int pin_posX = PA4;

constexpr int pin_negY = PA1;
constexpr int pin_posY = PB0;

constexpr int pin_negC = PA2;
constexpr int pin_posC = PB4;

void setupInterrupt() {

    pinMode(pin_negX, INPUT);
    pinMode(pin_posX, INPUT);

    pinMode(pin_negY, INPUT);
    pinMode(pin_posY, INPUT);

    pinMode(pin_negC, INPUT);
    pinMode(pin_posC, INPUT);


    NVIC_SetPriority(TIM3_IRQn, 0);
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_SetPriority(SPI1_IRQn, 5);
    NVIC_SetPriority(SPI2_IRQn, 4);
    NVIC_SetPriority(USART1_IRQn, 3);

    timer_profile.setOverflow(TICK_RATE_PROFILE, HERTZ_FORMAT);
    timer_profile.attachInterrupt(handleProfileInterrupt);
    timer_profile.resume();

    timer_pulse.setMode(1, TIMER_OUTPUT_COMPARE);
    timer_pulse.pause();
    timer_pulse.setOverflow(TICK_RATE_PULSE, HERTZ_FORMAT);
    timer_pulse.setCaptureCompare(1,1000);

    timer_pulse.attachInterrupt(1, handleCompareInterrupt);
    timer_pulse.attachInterrupt(handlePulseInterrupt);
    timer_pulse.refresh();
    timer_pulse.resume();
}

void handleSerial(Stream &port) {
    while (port.available()) {
        uint8_t b = port.read();
        protocolHandler.processReadByte(b);
    }

}

void registerCommands();

void logHex(const uint8_t* buf, size_t len) {
    Serial.print("HEX TX: ");
    for (size_t i = 0; i < len; i++) {
        if (buf[i] < 0x10) Serial.print("0");
        Serial.print(buf[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

void jumpToBootloader();
void saveID(uint8_t id);
uint8_t loadID();


void setup() {
    Serial1.begin(115200);
    EEPROM.begin();

    auto Id = loadID();
    protocolHandler.changeId(Id);

    //MT6835.init();
    hwX.init();
    hwY.init();
    hwC.init();

    delay(1000);

    setupInterrupt();
    registerCommands();
}

void loop() {
    handleSerial(Serial1);

    if (readyToReadEncoder) {
        readyToReadEncoder = false;
        limitsSwitch[0].negative = digitalRead(pin_negX);
        limitsSwitch[0].positive = digitalRead(pin_posX);

        limitsSwitch[1].negative = digitalRead(pin_negY);
        limitsSwitch[1].positive = digitalRead(pin_posY);

        limitsSwitch[2].negative = digitalRead(pin_negC);
        limitsSwitch[2].positive = digitalRead(pin_posC);
    }
}

void registerCommands() {

    CmdManager.addCommand(COMMAND::POWER_ENABLE, [&](const uint8_t* data, size_t len) {
        if (len < sizeof(CommandData)) return;
        auto* cmd = reinterpret_cast<const CommandData*>(data);
        Controller.powerEnable(cmd);

        auto status = Controller.getAxesInfo();
        auto pkg = protocolHandler.createFrame(COMMAND::POWER_ENABLE, status);
        auto* rawPkg = reinterpret_cast<uint8_t*>(&pkg);
        size_t pkgSize = sizeof(pkg);
        protocolHandler.sendFrame(rawPkg, pkgSize);
    });

    CmdManager.addCommand(COMMAND::MOVE_ABSOLUTE, [&](const uint8_t* data, size_t len) {
        if (len < sizeof(CommandData)) return;
        auto* cmd = reinterpret_cast<const CommandData*>(data);
        Controller.moveAbsolute(cmd);

        auto status = Controller.getAxesInfo();

        auto pkg = protocolHandler.createFrame(COMMAND::MOVE_ABSOLUTE, status);
        uint8_t* rawPkg = reinterpret_cast<uint8_t*>(&pkg);
        size_t pkgSize = sizeof(pkg);
        protocolHandler.sendFrame(rawPkg, pkgSize);
    });

    CmdManager.addCommand(COMMAND::MOVE_RELATIVE, [&](const uint8_t* data, size_t len) {
        if (len < sizeof(CommandData)) return;
        auto* cmd = reinterpret_cast<const CommandData*>(data);
        Controller.moveRelative(cmd);

        auto status = Controller.getAxesInfo();

        auto pkg = protocolHandler.createFrame(COMMAND::MOVE_RELATIVE, status);
        uint8_t* rawPkg = reinterpret_cast<uint8_t*>(&pkg);
        size_t pkgSize = sizeof(pkg);
        protocolHandler.sendFrame(rawPkg, pkgSize);
    });

    CmdManager.addCommand(COMMAND::MOVE_CONTINUE, [&](const uint8_t* data, size_t len) {
        if (len < sizeof(CommandData)) return;
        auto* cmd = reinterpret_cast<const CommandData*>(data);
        Controller.moveVelocity(cmd);

        auto status = Controller.getAxesInfo();

        auto pkg = protocolHandler.createFrame(COMMAND::MOVE_CONTINUE, status);
        uint8_t* rawPkg = reinterpret_cast<uint8_t*>(&pkg);
        size_t pkgSize = sizeof(pkg);
        protocolHandler.sendFrame(rawPkg, pkgSize);
    });

    CmdManager.addCommand(COMMAND::MOVE_HOME, [&](const uint8_t* data, size_t len) {
        if (len < sizeof(CommandData)) return;
        auto* cmd = reinterpret_cast<const CommandData*>(data);
        Controller.moveHome(cmd);

        auto status = Controller.getAxesInfo();

        auto pkg = protocolHandler.createFrame(COMMAND::MOVE_HOME, status);
        uint8_t* rawPkg = reinterpret_cast<uint8_t*>(&pkg);
        size_t pkgSize = sizeof(pkg);
        protocolHandler.sendFrame(rawPkg, pkgSize);
    });

    CmdManager.addCommand(COMMAND::HALT, [&](const uint8_t* data, size_t len) {
        if (len < sizeof(CommandData)) return;
        auto* cmd = reinterpret_cast<const CommandData*>(data);
        Controller.haltMove(cmd);

        auto status = Controller.getAxesInfo();

        auto pkg = protocolHandler.createFrame(COMMAND::HALT, status);
        uint8_t* rawPkg = reinterpret_cast<uint8_t*>(&pkg);
        size_t pkgSize = sizeof(pkg);
        protocolHandler.sendFrame(rawPkg, pkgSize);
    });

    CmdManager.addCommand(COMMAND::SET_POSITION, [&](const uint8_t* data, size_t len) {
        if (len < sizeof(CommandData)) return;
        auto* cmd = reinterpret_cast<const CommandData*>(data);
        Controller.setCurrentPositions(cmd);

        auto status = Controller.getAxesInfo();

        auto pkg = protocolHandler.createFrame(COMMAND::SET_POSITION, status);
        uint8_t* rawPkg = reinterpret_cast<uint8_t*>(&pkg);
        size_t pkgSize = sizeof(pkg);
        protocolHandler.sendFrame(rawPkg, pkgSize);
    });

    CmdManager.addCommand(COMMAND::GET_STATUS, [&](const uint8_t* data, size_t len) {
        if (len < 1) return;
        auto status = Controller.getAxesInfo();

        auto pkg = protocolHandler.createFrame(COMMAND::GET_STATUS, status);
        uint8_t* rawPkg = reinterpret_cast<uint8_t*>(&pkg);
        size_t pkgSize = sizeof(pkg);
        protocolHandler.sendFrame(rawPkg, pkgSize);
    });

    CmdManager.addCommand(COMMAND::RESET_ERROR, [&](const uint8_t* data, size_t len) {
        if (len < sizeof(CommandData)) return;

        auto* cmd = reinterpret_cast<const CommandData*>(data);
        Controller.recoverAxes(cmd);

        auto status = Controller.getAxesInfo();

        auto pkg = protocolHandler.createFrame(COMMAND::RESET_ERROR, status);
        uint8_t* rawPkg = reinterpret_cast<uint8_t*>(&pkg);
        size_t pkgSize = sizeof(pkg);
        protocolHandler.sendFrame(rawPkg, pkgSize);
    });

    CmdManager.addCommand(COMMAND::GET_FEEDBACK, [&](const uint8_t* data, size_t len) {
        if (len < 1) return;
        auto info = Controller.getAxesFeedback();

        auto pkg = protocolHandler.createFrame(COMMAND::GET_FEEDBACK, info);
        uint8_t* rawPkg = reinterpret_cast<uint8_t*>(&pkg);
        size_t pkgSize = sizeof(pkg);
        protocolHandler.sendFrame(rawPkg, pkgSize);
    });

    CmdManager.addCommand(COMMAND::SET_HOMING, [&](const uint8_t* data, size_t len) {
        if (data == nullptr || len != sizeof(AxisHomingConfig) * MAX_NUMBER_DEVICE) {
            // auto pkg = protocolHandler.createFrame(COMMAND::SET_HOMING, CommandAck{0});
            // protocolHandler.sendFrame(reinterpret_cast<uint8_t*>(&pkg), sizeof(pkg));
            return;
        }
        const auto* conf = reinterpret_cast<const AxisHomingConfig*>(data);
        Controller.setHomingConfig(conf);
        auto status = Controller.getAxesInfo();

        auto pkg = protocolHandler.createFrame(COMMAND::SET_HOMING, status);
        protocolHandler.sendFrame(reinterpret_cast<uint8_t*>(&pkg), sizeof(pkg));
    });

    CmdManager.addCommand(COMMAND::SET_LIMITS, [&](const uint8_t* data, size_t len) {
        if (data == nullptr || len != sizeof(AxisSoftwareLimit) * 3) {
            // auto pkg = protocolHandler.createFrame(COMMAND::SET_LIMITS, CommandAck{0});
            // protocolHandler.sendFrame(reinterpret_cast<uint8_t*>(&pkg), sizeof(pkg));
            return;
        }
        const auto* Limits = reinterpret_cast<const AxisSoftwareLimit*>(data);
        Controller.setAxisSoftwareLimit(Limits);
        auto status = Controller.getAxesInfo();
        auto pkg = protocolHandler.createFrame(COMMAND::SET_LIMITS, status);
        protocolHandler.sendFrame(reinterpret_cast<uint8_t*>(&pkg), sizeof(pkg));
    });

    CmdManager.addCommand(COMMAND::CHANGE_ID_DEVICE, [&](const uint8_t* data, size_t len) {
        if (data == nullptr || len != 1) return;

        uint8_t newId = data[0];
        auto status = Controller.getAxesInfo();

        auto pkg = protocolHandler.createFrame(COMMAND::CHANGE_ID_DEVICE,  status);
        uint8_t* rawPkg = reinterpret_cast<uint8_t*>(&pkg);
        size_t pkgSize = sizeof(pkg);
        protocolHandler.sendFrame(rawPkg, pkgSize);
        if (protocolHandler.getId() != newId) {
            protocolHandler.changeId(newId);
            saveID(newId);
        }

    });

    CmdManager.addCommand(COMMAND::JUMP_TO_BOOTLOADER, [&](const uint8_t* data, size_t len) {
        jumpToBootloader();
   });
}

void jumpToBootloader() {
        __disable_irq();
        SysTick->CTRL = 0;
        SysTick->LOAD = 0;
        SysTick->VAL = 0;

        for (int i = 0; i < 8; i++) {
            NVIC->ICER[i] = 0xFFFFFFFF;
            NVIC->ICPR[i] = 0xFFFFFFFF;
        }

        __enable_irq();

        uint32_t bootloaderAddress = 0x1FFF0000;
        __set_MSP(*((__IO uint32_t*)bootloaderAddress));

        void (*SysMemBootloader)(void) = (void (*)(void))(*((__IO uint32_t*)(bootloaderAddress + 4)));
        SysMemBootloader();
}

void saveID(uint8_t id) {
    EEPROM.write(0, id);
}

uint8_t loadID() {
    uint8_t id = EEPROM.read(0);

    if (id == 0xFF || id == 0) {
        id = DEFAULT_ID;
        saveID(id);
    }
    return id;
}