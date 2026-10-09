#include "../include/PulseGenerator.hpp"
#include "../include/AxisHardware.hpp"


PulseDriver::PulseDriver(AxisHardware* hw, float tickPeriod, float cycleTime, bool invertDir)
    : Hardware(hw), ticksPerCycle(static_cast<int32_t>(cycleTime / tickPeriod + 0.5f)), invert(invertDir) {}

int32_t PulseDriver::getCounterPulse() const{
    return counterPulse;
}

int32_t PulseDriver::getTargetSteps() const {
    return targetSteps;
}

void PulseDriver::setTargetSteps(int32_t steps) {
    targetSteps = steps;
}

void PulseDriver::handleStepLow() {
    if (!pulseHigh) return;
    Hardware->setStep(false);
    pulseHigh = false;
}


void PulseDriver::setCounterPulse(const int32_t pulse) {
    counterPulse = pulse;
    targetSteps = pulse;
    stepsThisCycle = 0;
}


void PulseDriver::run() {
    if (phase == 0) {
        // начало цикла планировщика: сколько шагов выдать за следующие ticksPerCycle тиков
        int32_t delta = targetSteps - counterPulse;
        if (delta != 0) {
            int8_t dir = delta > 0 ? 1 : -1;
            if (dir != lastDir) {
                lastDir = dir;
                Hardware->setDir((dir > 0) != invert);
                holdStep = true;               // первый шаг — со следующего тика: время установки DIR
            }
        }
        stepsThisCycle = delta >= 0 ? delta : -delta;
        error = 0;
    }
    if (++phase == ticksPerCycle) phase = 0;

    if (holdStep) {
        holdStep = false;
        return;
    }
    error += stepsThisCycle;
    if (stepsThisCycle > 0 && error >= ticksPerCycle) {
        error -= ticksPerCycle;
        Hardware->setStep(true);
        pulseHigh = true;
        counterPulse += lastDir;
    }
}

////==========================================================================
