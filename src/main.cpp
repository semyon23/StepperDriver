#include "../include/AxesManager.hpp"
#include "../include/AxisHardware.hpp"
#include "../include/ControlLock.h"
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

struct FilteredLimit {
    uint8_t negCounter = 0;
    uint8_t posCounter = 0;
    const uint8_t DEBOUNCE_THRESHOLD = 10;
};

FilteredLimit limitsFilter[MAX_NUMBER_DEVICE];


AxisLimitSwitch limitsSwitch[MAX_NUMBER_DEVICE] = {
    {.negative = false, .positive = false},
    {.negative = false, .positive = false},
    {.negative = false, .positive = false}
};

AxisHardware hwX(config[0]);
AxisHardware hwY(config[1]);
AxisHardware hwC(config[2]);

AxisControl axisX(1, &hwX, &limitsSwitch[0], CYCLE_TIME, TICK_RATE, true);
AxisControl axisY(2, &hwY, &limitsSwitch[1], CYCLE_TIME, TICK_RATE, true);
AxisControl axisC(3, &hwC, &limitsSwitch[2], CYCLE_TIME, TICK_RATE, false);


CommandManager CmdManager;

ProtocolHandler protocolHandler(&CmdManager, DEFAULT_ID);

HardwareTimer timer_profile(TIM2);
HardwareTimer timer_pulse(TIM3);

AxesManager Controller(&axisX, &axisY, &axisC);

volatile bool readyToReadEncoder = false;

constexpr PinName pin_negX = PA_0;
constexpr PinName pin_posX = PA_4;

constexpr PinName pin_negY = PA_1;
constexpr PinName pin_posY = PB_0;

constexpr PinName pin_negC = PA_2;
constexpr PinName pin_posC = PB_4;


void handleProfileInterrupt() {
    // Лямбда-функция для фильтрации
    auto applyFilter = [](bool rawValue, uint8_t &counter, const uint8_t threshold, bool &targetLimit) {
        if (rawValue) {
            if (counter < threshold) counter++;
        } else {
            if (counter > 0) counter--;
        }

        if (counter == threshold) targetLimit = true;
        else if (counter == 0)     targetLimit = false;
    };

    // Ось X: Быстрое чтение PinName + фильтр
    applyFilter(digitalReadFast(pin_negX), limitsFilter[0].negCounter, limitsFilter[0].DEBOUNCE_THRESHOLD, limitsSwitch[0].negative);
    applyFilter(digitalReadFast(pin_posX), limitsFilter[0].posCounter, limitsFilter[0].DEBOUNCE_THRESHOLD, limitsSwitch[0].positive);

    // Ось Y: Быстрое чтение PinName + фильтр
    applyFilter(digitalReadFast(pin_negY), limitsFilter[1].negCounter, limitsFilter[1].DEBOUNCE_THRESHOLD, limitsSwitch[1].negative);
    applyFilter(digitalReadFast(pin_posY), limitsFilter[1].posCounter, limitsFilter[1].DEBOUNCE_THRESHOLD, limitsSwitch[1].positive);

    // Ось C: Быстрое чтение PinName + фильтр
    applyFilter(digitalReadFast(pin_negC), limitsFilter[2].negCounter, limitsFilter[2].DEBOUNCE_THRESHOLD, limitsSwitch[2].negative);
    applyFilter(digitalReadFast(pin_posC), limitsFilter[2].posCounter, limitsFilter[2].DEBOUNCE_THRESHOLD, limitsSwitch[2].positive);



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




// constexpr int pin_negX = PA0;
// constexpr int pin_posX = PA4;
//
// constexpr int pin_negY = PA1;
// constexpr int pin_posY = PB0;
//
// constexpr int pin_negC = PA2;
// constexpr int pin_posC = PB4;

void setupInterrupt() {

    pinMode(pin_negX, INPUT_PULLUP);
    pinMode(pin_posX, INPUT_PULLUP);

    pinMode(pin_negY, INPUT_PULLUP);
    pinMode(pin_posY, INPUT_PULLUP);

    pinMode(pin_negC, INPUT_PULLUP);
    pinMode(pin_posC, INPUT_PULLUP);


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
    protocolHandler.setId(Id);

    hwX.init();
    hwY.init();
    hwC.init();

    delay(1000);

    setupInterrupt();
    registerCommands();
}

void loop() {
    handleSerial(Serial1);

    // if (readyToReadEncoder) {
    //     readyToReadEncoder = false;
    //     limitsSwitch[0].negative = digitalRead(pin_negX);
    //     limitsSwitch[0].positive = digitalRead(pin_posX);
    //
    //     limitsSwitch[1].negative = digitalRead(pin_negY);
    //     limitsSwitch[1].positive = digitalRead(pin_posY);
    //
    //     limitsSwitch[2].negative = digitalRead(pin_negC);
    //     limitsSwitch[2].positive = digitalRead(pin_posC);
    // }
}
template <typename T>
void reply(COMMAND::ID id, const T& data) {
    auto pkg = protocolHandler.createFrame(id, data);
    protocolHandler.sendFrame(reinterpret_cast<uint8_t*>(&pkg), sizeof(pkg));
}

template <void (AxesManager::*Action)(const CommandData*)>
    void axesCommand(COMMAND::ID id, const uint8_t* data, size_t len) {
    if (len < sizeof(CommandData)) return;
    CommandData cmd;
    memcpy(&cmd, data, sizeof(cmd));
    StatusData status;
    {
        ControlLock lock;
        (Controller.*Action)(&cmd);
        status = Controller.getAxesInfo();
    }
    reply(id, status);
}

void registerCommands() {
    CmdManager.addCommand(COMMAND::POWER_ENABLE,   [](const uint8_t* d, size_t n)
        { axesCommand<&AxesManager::powerEnable>(COMMAND::POWER_ENABLE, d, n); });
    CmdManager.addCommand(COMMAND::MOVE_ABSOLUTE,  [](const uint8_t* d, size_t n)
        { axesCommand<&AxesManager::moveAbsolute>(COMMAND::MOVE_ABSOLUTE, d, n); });
    CmdManager.addCommand(COMMAND::MOVE_RELATIVE,  [](const uint8_t* d, size_t n)
        { axesCommand<&AxesManager::moveRelative>(COMMAND::MOVE_RELATIVE, d, n); });
    CmdManager.addCommand(COMMAND::MOVE_CONTINUE,  [](const uint8_t* d, size_t n)
        { axesCommand<&AxesManager::moveVelocity>(COMMAND::MOVE_CONTINUE, d, n); });
    CmdManager.addCommand(COMMAND::MOVE_HOME,      [](const uint8_t* d, size_t n)
        { axesCommand<&AxesManager::moveHome>(COMMAND::MOVE_HOME, d, n); });
    CmdManager.addCommand(COMMAND::HALT,           [](const uint8_t* d, size_t n)
        { axesCommand<&AxesManager::haltMove>(COMMAND::HALT, d, n); });
    CmdManager.addCommand(COMMAND::EMERGENCY_STOP, [](const uint8_t* d, size_t n)
        { axesCommand<&AxesManager::hardStop>(COMMAND::EMERGENCY_STOP, d, n); });
    CmdManager.addCommand(COMMAND::SET_POSITION,   [](const uint8_t* d, size_t n)
        { axesCommand<&AxesManager::setCurrentPositions>(COMMAND::SET_POSITION, d, n); });
    CmdManager.addCommand(COMMAND::RESET_ERROR,    [](const uint8_t* d, size_t n)
        { axesCommand<&AxesManager::recoverAxes>(COMMAND::RESET_ERROR, d, n); });

    CmdManager.addCommand(COMMAND::GET_STATUS, [](const uint8_t*, size_t n) {
        if (n < 1) return;
        StatusData s;
        { ControlLock lock; s = Controller.getAxesInfo(); }
        reply(COMMAND::GET_STATUS, s);
    });
    CmdManager.addCommand(COMMAND::GET_FEEDBACK, [](const uint8_t*, size_t n) {
        if (n < 1) return;
        FeedbackData f;
        { ControlLock lock; f = Controller.getAxesFeedback(); }
        reply(COMMAND::GET_FEEDBACK, f);
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
             protocolHandler.setId(newId);
             saveID(newId);
         }
     });

     CmdManager.addCommand(COMMAND::JUMP_TO_BOOTLOADER, [&](const uint8_t* data, size_t len) {
         jumpToBootloader();
    });

}

void jumpToBootloader() {
    // __disable_irq();
    //
    // RCC->CR |= RCC_CR_HSION;
    // while ((RCC->CR & RCC_CR_HSIRDY) == 0);
    //
    // RCC->CFGR &= ~RCC_CFGR_SW;
    // while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
    //
    // // Выключаем PLL, HSE и CSS
    // RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_HSEON | RCC_CR_CSSON);
    //з
    // // Сбрасываем конфигурацию PLL
    // RCC->PLLCFGR = 0x24003010; // Дефолтное значение регистра для STM32F4
    //
    // RCC->CFGR &= 0x00000000;
    // RCC->CIR = 0x00000000;
    //
    // SysTick->CTRL = 0;
    // SysTick->LOAD = 0;
    // SysTick->VAL = 0;
    //
    // // Сброс NVIC
    // for (int i = 0; i < 8; i++) {
    //     NVIC->ICER[i] = 0xFFFFFFFF;
    //     NVIC->ICPR[i] = 0xFFFFFFFF;
    // }
    //
    // uint32_t bootloaderAddress = 0x1FFF0000;
    // __set_MSP(*((__IO uint32_t *) bootloaderAddress));
    //
    // void (*SysMemBootloader)(void) = (void (*)(void)) (*((__IO uint32_t *) (bootloaderAddress + 4)));
    // SysMemBootloader();
    //
    // while (1);
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