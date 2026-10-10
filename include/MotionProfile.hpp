
#ifndef UNTITLED3_TRAPEZOIDALMOTIONPROFILE_HPP
#define UNTITLED3_TRAPEZOIDALMOTIONPROFILE_HPP
#include <Arduino.h>


namespace MotionProfile {
    class TrapezoidalProfile{
    public:
        // maxStepRate — потолок скорости: PulseDriver выдаёт не больше одного шага за тик (50 кГц)
        explicit TrapezoidalProfile(float ct, float maxStepRate = 48000.0f);
        void generate_profile();
        void run();
        void stop_profile();
        void hard_stop_profile();
        void continue_profile();
        void setAcceleration(float acc);
        void setDeceleration(float dec);
        void setVelocity(float vel);
        void setPosition(float pos);
        void setCurrentPosition(float pos);
        void setDirection(int8_t dir);
        [[nodiscard]]float getTargetPosition() const;
        [[nodiscard]]int8_t getDirectionSign() const;
        [[nodiscard]]float getPosition() const;
        [[nodiscard]]float getVelocity() const;
        [[nodiscard]]bool inMotion() const;
        [[nodiscard]]int32_t getPositionSteps() const;

    private:
        enum class Mode : uint8_t { IDLE, DISCRETE, CONTINUOUS, STOPPING };

        int64_t perCycle(float perSecond) const;
        int64_t perCycle2(float perSecond2) const;
        int64_t brakingDistance(int64_t v) const;
        bool canStop(int64_t v, int64_t remaining) const;
        int64_t maxStoppableSpeed(int64_t r, int64_t limit) const;
        void runDiscrete();
        void rampVelocity(int64_t target, int64_t up, int64_t down);
        void setTarget(int64_t target);
        void publish();

        float cycle_time{0.0f};
        int64_t maxStepsPerCycle{0};

        volatile Mode mode{Mode::IDLE};
        int64_t current_position{0};
        int64_t current_velocity{0};
        int64_t targetPosition{0};
        int64_t maxVelocity{0};
        int64_t maxAcceleration{1};
        int64_t maxDeceleration{1};
        int8_t target_direction{1};
        int8_t directionSign{1};

        // 32-битные копии для чтения из loop(): запись 32 бит атомарна, 64 бит — нет
        volatile int32_t positionSteps{0};
        volatile float positionReport{0.0f};
        volatile float velocityReport{0.0f};
        volatile float targetReport{0.0f};
    };
} // MotionProfile

#endif //UNTITLED3_TRAPEZOIDALMOTIONPROFILE_HPP
