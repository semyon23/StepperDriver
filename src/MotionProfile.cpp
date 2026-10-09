#include "../include/MotionProfile.hpp"

#include <algorithm>
#include <cmath>

namespace MotionProfile {
    namespace {
        constexpr float kOne = 4294967296.0f;                   // 1 шаг = 2^32
        constexpr int64_t kStep = int64_t{1} << 32;
        constexpr int64_t kHalfStep = int64_t{1} << 31;
        constexpr int64_t kMaxBrakeCycles = int64_t{1} << 25;   // ~4,6 ч торможения — дальше защита от переполнения
        constexpr int64_t kFarAway = INT64_MAX / 4;
        constexpr float kMaxSteps = 2147483520.0f;              // наибольший float меньше 2^31

        int64_t toFixed(float steps) {
            steps = std::max(-kMaxSteps, std::min(steps, kMaxSteps));
            float whole = std::floor(steps);
            return static_cast<int64_t>(whole) * kStep + static_cast<int64_t>((steps - whole) * kOne);
        }

        float toFloat(int64_t fixed) {
            return static_cast<float>(fixed) / kOne;
        }
    }

    TrapezoidalProfile::TrapezoidalProfile(float ct, float maxStepRate)
        : cycle_time(ct), maxStepsPerCycle(static_cast<int64_t>(maxStepRate * ct * kOne)) {}

    bool TrapezoidalProfile::inMotion() const {
        return mode != Mode::IDLE;
    }

    float TrapezoidalProfile::getTargetPosition() const{
        return targetReport;
    }

    void TrapezoidalProfile::setDirection(int8_t dir) {
        target_direction = dir;
    }

    // Направление текущего движения; в покое — направление последней команды.
    int8_t TrapezoidalProfile::getDirectionSign() const {
        float v = velocityReport;
        if (v > 0.0f) return 1;
        if (v < 0.0f) return -1;
        return directionSign;
    }

    void TrapezoidalProfile::setCurrentPosition(float pos) {
        int64_t newPosition = toFixed(pos);
        int64_t shift = newPosition - current_position;
        current_position = newPosition;
        if (mode == Mode::IDLE) {
            setTarget(newPosition);
        } else if (mode == Mode::STOPPING) {
            setTarget(targetPosition + shift);
        }
        // DISCRETE: цель абсолютная, путь к ней пересчитывается каждый цикл
        publish();
    }

    void TrapezoidalProfile::setPosition(float pos) {
        setTarget(toFixed(std::round(pos)));
    }

    void TrapezoidalProfile::setVelocity(float vel) {
        maxVelocity = perCycle(vel);
    }

    void TrapezoidalProfile::setAcceleration(float acc) {
        maxAcceleration = perCycle2(acc);
    }
    void TrapezoidalProfile::setDeceleration(float dec) {
        maxDeceleration = perCycle2(dec);
    }

    float TrapezoidalProfile::getPosition() const {
        return positionReport;
    }

    float TrapezoidalProfile::getVelocity() const {
        return velocityReport;
    }

    int32_t TrapezoidalProfile::getPositionSteps() const {
        return positionSteps;
    }

    void TrapezoidalProfile::hard_stop_profile() {
        current_velocity = 0;
        mode = Mode::IDLE;
        setTarget(current_position);
        publish();
    }

    void TrapezoidalProfile::stop_profile() {
        if (mode == Mode::IDLE || current_velocity == 0) {
            hard_stop_profile();
            return;
        }
        // точка остановки известна заранее: торможение на maxDeceleration за цикл — это brakingDistance
        int64_t speed = current_velocity > 0 ? current_velocity : -current_velocity;
        int64_t distance = brakingDistance(speed);
        if (distance < kFarAway) {
            setTarget(current_position + (current_velocity > 0 ? distance : -distance));
        }
        mode = Mode::STOPPING;
    }

    void TrapezoidalProfile::continue_profile() {
        directionSign = target_direction;
        mode = Mode::CONTINUOUS;
    }

    void TrapezoidalProfile::generate_profile() {
        int64_t diff = targetPosition - current_position;
        if (diff == 0 && current_velocity == 0) {
            mode = Mode::IDLE;
            return;
        }
        if (diff != 0) {
            directionSign = diff > 0 ? 1 : -1;
        } else {
            directionSign = current_velocity > 0 ? 1 : -1;
        }
        mode = Mode::DISCRETE;
    }

    void TrapezoidalProfile::run() {
        switch (mode) {
            case Mode::IDLE:
                return;
            case Mode::DISCRETE:
                runDiscrete();
                break;
            case Mode::CONTINUOUS:
                // как в прежней версии: и разгон, и разворот — с ускорением
                rampVelocity(target_direction * maxVelocity, maxAcceleration, maxAcceleration);
                current_position += current_velocity;
                break;
            case Mode::STOPPING:
                rampVelocity(0, maxAcceleration, maxDeceleration);
                current_position += current_velocity;
                if (current_velocity == 0) {
                    mode = Mode::IDLE;
                    setTarget(current_position);
                }
                break;
        }
        publish();
    }

    int64_t TrapezoidalProfile::perCycle(float perSecond) const {
        float v = std::fabs(perSecond) * cycle_time * kOne;
        if (std::isnan(v)) return 0;
        if (v >= static_cast<float>(maxStepsPerCycle)) return maxStepsPerCycle;
        return static_cast<int64_t>(v);
    }

    int64_t TrapezoidalProfile::perCycle2(float perSecond2) const {
        float a = std::fabs(perSecond2) * cycle_time * cycle_time * kOne;
        if (!(a >= 1.0f)) return 1;                       // ноль и NaN: минимальное ненулевое значение
        if (a >= 4.0e18f) return int64_t{4000000000000000000};
        return static_cast<int64_t>(a);
    }

    // Путь, который ось проедет, если со следующего цикла тормозить на maxDeceleration за цикл:
    // (v - dec) + (v - 2·dec) + … — точная дискретная сумма, а не v²/2a.
    int64_t TrapezoidalProfile::brakingDistance(int64_t v) const {
        if (v <= 0) return 0;
        int64_t n = v / maxDeceleration;
        if (n >= kMaxBrakeCycles) return kFarAway;
        return n * v - maxDeceleration * (n * (n + 1) / 2);
    }

    bool TrapezoidalProfile::canStop(int64_t v, int64_t remaining) const {
        return v <= remaining && remaining - v >= brakingDistance(v);
    }

    void TrapezoidalProfile::runDiscrete() {
        int64_t diff = targetPosition - current_position;
        if (diff == 0 && current_velocity == 0) {
            mode = Mode::IDLE;
            return;
        }
        // считаем «вдоль направления к цели»: r — сколько осталось, u — скорость к цели
        int64_t s = (diff > 0 || (diff == 0 && current_velocity > 0)) ? 1 : -1;
        int64_t r = s * diff;
        int64_t u = s * current_velocity;

        int64_t next;
        if (u < 0) {
            next = std::min<int64_t>(u + maxDeceleration, 0);      // едем от цели — сначала остановиться
        } else {
            int64_t lo = std::max<int64_t>(u - maxDeceleration, 0); // сильнее тормозить нельзя
            int64_t hi = (u <= maxVelocity) ? std::min(u + maxAcceleration, maxVelocity)
                                            : std::max(u - maxDeceleration, maxVelocity);
            // Наибольшая скорость, с которой ещё успеваем остановиться в цели: v + v²/2dec ≈ r.
            // Оценка во float (важна только относительная точность), проверка — точная, в целых.
            float d = static_cast<float>(maxDeceleration);
            float est = std::sqrt(2.0f * d * static_cast<float>(r) + 0.25f * d * d) - 0.5f * d;
            int64_t want = est >= static_cast<float>(hi) ? hi : static_cast<int64_t>(est * 0.9999f);
            next = std::max(lo, std::min(std::max<int64_t>(want, 1), r));
            while (next > lo && !canStop(next, r)) {
                next = lo + (next - lo) / 2;
            }
        }
        current_velocity = s * next;
        current_position += current_velocity;
    }

    // up — когда скорость по модулю растёт, down — когда падает (в том числе до разворота)
    void TrapezoidalProfile::rampVelocity(int64_t target, int64_t up, int64_t down) {
        if (current_velocity < target) {
            current_velocity = std::min(current_velocity + (current_velocity < 0 ? down : up), target);
        } else if (current_velocity > target) {
            current_velocity = std::max(current_velocity - (current_velocity > 0 ? down : up), target);
        }
    }

    void TrapezoidalProfile::setTarget(int64_t target) {
        targetPosition = target;
        targetReport = toFloat(target);
    }

    void TrapezoidalProfile::publish() {
        positionSteps = static_cast<int32_t>((current_position + kHalfStep) >> 32);
        positionReport = toFloat(current_position);
        velocityReport = static_cast<float>(current_velocity) / (kOne * cycle_time);
    }
} // MotionProfile
