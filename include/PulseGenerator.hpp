
#ifndef UNTITLED3_MOTIONEXECUTOR_HPP
#define UNTITLED3_MOTIONEXECUTOR_HPP

#include <Arduino.h>

class AxisHardware;

// Выдаёт столько шагов, сколько нужно до целой позиции планировщика, равномерно
// по тикам цикла (алгоритм Брезенхэма). Счётчик шагов догоняет позицию точно,
// ошибка не накапливается. Инверсия направления — только на пине DIR,
// счётчик идёт в тех же координатах, что и планировщик.
class PulseDriver {
public:
    PulseDriver(AxisHardware* hw, float tickPeriod, float cycleTime, bool invertDir = false);
    void run();                                // TIM3, каждый тик (50 кГц)
    void handleStepLow();                      // TIM3 compare, середина тика
    int32_t getCounterPulse() const;
    int32_t getTargetSteps() const;
    void setTargetSteps(int32_t steps);        // TIM2, после MotionPlanner.run()
    void setCounterPulse(const int32_t pulse); // только в покое: позиция и цель сразу
private:

    AxisHardware* Hardware;
    int32_t ticksPerCycle;
    bool invert;

    volatile int32_t targetSteps{0};
    volatile int32_t counterPulse{0};
    int32_t stepsThisCycle{0};
    int32_t error{0};
    int32_t phase{0};
    int8_t lastDir{0};                         // 0 — пин DIR ещё ни разу не выставлялся
    bool holdStep{false};
    bool pulseHigh{false};
};


#endif //UNTITLED3_MOTIONEXECUTOR_HPP
