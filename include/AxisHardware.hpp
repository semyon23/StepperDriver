#ifndef UNTITLED3_AXISHARDWARE_HPP
#define UNTITLED3_AXISHARDWARE_HPP


struct AxisPins {
    GPIO_TypeDef *port;
    uint16_t pin;
};

struct AxisConfig {
    AxisPins step;
    AxisPins dir;
    AxisPins enable;
    AxisPins chipSelect;
    AxisPins fault;
    AxisPins pulseHome;
};

class AxisHardware {

public:
    AxisHardware(const AxisConfig& config) : _cfg(config) {}

    void init() {
        auto safeEnable = [&](GPIO_TypeDef* port) {
            if (port) enableClock(port);
        };

        safeEnable(_cfg.step.port);
        safeEnable(_cfg.dir.port);
        safeEnable(_cfg.enable.port);
        safeEnable(_cfg.fault.port);
        safeEnable(_cfg.chipSelect.port);

        GPIO_InitTypeDef io = {0};

        if (_cfg.step.port) {
            io.Pin = _cfg.step.pin;
            io.Mode = GPIO_MODE_OUTPUT_PP;
            io.Pull = GPIO_NOPULL;
            io.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
            HAL_GPIO_Init(_cfg.step.port, &io);
        }

        if (_cfg.dir.port) {
            io.Pin = _cfg.dir.pin;
            io.Mode = GPIO_MODE_OUTPUT_PP;
            io.Pull = GPIO_NOPULL;
            io.Speed = GPIO_SPEED_FREQ_LOW;
            HAL_GPIO_Init(_cfg.dir.port, &io);
        }

        if (_cfg.enable.port) {
            io.Pin = _cfg.enable.pin;
            io.Mode = GPIO_MODE_OUTPUT_PP;
            io.Pull = GPIO_NOPULL;
            io.Speed = GPIO_SPEED_FREQ_LOW;
            HAL_GPIO_Init(_cfg.enable.port, &io);
        }

        if (_cfg.chipSelect.port) {
            io.Pin = _cfg.chipSelect.pin;
            io.Mode = GPIO_MODE_OUTPUT_PP;
            io.Pull = GPIO_NOPULL;
            io.Speed = GPIO_SPEED_FREQ_LOW;
            HAL_GPIO_Init(_cfg.chipSelect.port, &io);
        }

        io.Mode = GPIO_MODE_INPUT;

        auto initInput = [&](const AxisPins& p) {
            if (!p.port) return;
            GPIO_InitTypeDef ioInput = {0};
            ioInput.Pin = p.pin;
            ioInput.Mode = GPIO_MODE_INPUT;
            ioInput.Pull = GPIO_PULLDOWN;
            HAL_GPIO_Init(p.port, &ioInput);
        };

        initInput(_cfg.fault);
        initInput(_cfg.pulseHome);

        setChipSelect(true);
        setEnable(true);
    }

    void setChipSelect(bool state) {
        if (!_cfg.chipSelect.port) return;
        if (state) _cfg.chipSelect.port->BSRR = _cfg.chipSelect.pin;
        else       _cfg.chipSelect.port->BSRR = (uint32_t)_cfg.chipSelect.pin << 16;
    }

    void setEnable(bool state) const {
        if (!_cfg.enable.port) return;
        if (!state)_cfg.enable.port->BSRR = _cfg.enable.pin;
        else  _cfg.enable.port->BSRR = (uint32_t)_cfg.enable.pin << 16;;
        // if (!_cfg.enable.port) return;
        // HAL_GPIO_WritePin(_cfg.enable.port, _cfg.enable.pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    };

    void setDir(bool forward) const {
        if (!_cfg.dir.port) return;
        if (forward) _cfg.dir.port->BSRR = _cfg.dir.pin;
        else            _cfg.dir.port->BSRR = (uint32_t)_cfg.dir.pin << 16;
    }

    void setStep(bool state) const {
        if (!_cfg.step.port) return;
        if (state) _cfg.step.port->BSRR = _cfg.step.pin;
        else       _cfg.step.port->BSRR = (uint32_t)_cfg.step.pin << 16;
    }


    bool faultActive() const {
        if (!_cfg.fault.port) return false;
        bool raw = (HAL_GPIO_ReadPin(_cfg.fault.port, _cfg.fault.pin) == GPIO_PIN_RESET);
        return raw;
    }

    bool pulseHome() const {
        if (!_cfg.pulseHome.port) return false;
        bool raw = (HAL_GPIO_ReadPin(_cfg.pulseHome.port, _cfg.pulseHome.pin) == GPIO_PIN_RESET);
        return raw;
    }
private:
    AxisConfig _cfg;

    void enableClock(GPIO_TypeDef* port) {
        if (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
        else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
        else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    }
};

#endif //UNTITLED3_AXISHARDWARE_HPP