#ifndef UNTITLED3_AXISCONTROLLER_HPP
#define UNTITLED3_AXISCONTROLLER_HPP
#include "MotionProfile.hpp"
#include "PulseGenerator.hpp"
#include "../include/AxisHardware.hpp"


enum ERROR_ID : uint8_t {
    NO_ERROR,
    EMERGENCY_STOP,
    DRIVE_OVERLOAD,
    HARDWARE_NEG_LS_ACTIVE,
    HARDWARE_POS_LS_ACTIVE,
    HOMING_TIMEOUT
};

enum HOMING_MODE : int8_t {
    NEGATIVE_LIMIT    = -1,
    POSITIVE_LIMIT    = 1,
    DIRECT_HOME       = 0,
};

#pragma pack(push, 1)
    struct AxisFlags {
        bool inMotion{false};
        bool inTarget{false};
        bool error{false};
    };

    struct AxisSoftwareLimit {
        float minPosition{0.0f};
        float maxPosition{0.0f};
        float velocity{0.0f};
        float acceleration{0.0f};
        float deceleration{0.0f};
    };

    struct AxisHomingConfig {
        HOMING_MODE mode{HOMING_MODE::DIRECT_HOME};
        float velocity{0.0f};
        float crowl_velocity{0.0f};
        float acceleration{0.0f};
        float deceleration{0.0f};
        float position_offset{0.0f};
        uint8_t timeout{0};
    };
#pragma pack(pop)

enum DIRECTION : int8_t {
    NEGATIVE    = -1,
    POSITIVE    = 1,
};


class AxisHardware;

class AxisControl;

struct MoveSettings {
    float distance{0.0f};
    float velocity{0.0f};
    float acceleration{0.0f};
    float deceleration{0.0f};
    DIRECTION direction{DIRECTION::POSITIVE};
};


struct AxisLimitSwitch {
    bool negative{false};
    bool positive{false};
};

enum class AxisStateId : uint8_t {
    DISABLED = 0,
    STANDSTILL = 1,
    DISCRETE_MOTION = 2,
    CONTINUE_MOTION = 3,
    STOPPING = 4,
    HOMING = 5,
    ERROR = 6
};

class AxisState {
public:
    virtual void setup(AxisControl* axis) {}
    virtual void action(AxisControl* axis) = 0;
    virtual void cleanup(AxisControl* axis) {}
    virtual ~AxisState() = default;
};

class DiscreteMotionState : public AxisState {
public:
    void action(AxisControl* axis) override;
    void setup(AxisControl* axis) override;
};

class ContinueMotionState : public AxisState {
public:
    void setup(AxisControl* axis) override;
    void action(AxisControl* axis) override;
};

class StoppingState : public AxisState {
public:
    void setup(AxisControl* axis) override;
    void action(AxisControl* axis) override;
};

class StandstillState : public AxisState {
public:
    void setup(AxisControl* axis) override;
    void action(AxisControl* axis) override;
};

class DisabledState : public AxisState {
public:
    void setup(AxisControl* axis) override;
    void action(AxisControl* axis) override;
};

class HomingState : public AxisState {
public:
    void setup(AxisControl* axis) override;
    void action(AxisControl* axis) override;
};

class ErrorState : public AxisState {
public:
    void setup(AxisControl* axis) override;
    void action(AxisControl* axis) override;
    void cleanup(AxisControl* axis) override;
};

class AxisControl {
    friend class DisabledState;
    friend class StandstillState;
    friend class DiscreteMotionState;
    friend class ContinueMotionState;
    friend class StoppingState;
    friend class HomingState;
    friend class ErrorState;
public:
    AxisControl(uint8_t id, AxisHardware* hw, AxisLimitSwitch* ls, float cycleTime, float tickRate, bool invertDir = false);
    void update();
    void recover();
    void enableDriver();
    void disableDriver();
    void emergencyStop();
    void moveRelative(float distance, float vel, float acc, float dec);
    void moveAbsolute(float pos, float vel, float acc, float dec);
    void moveVelocity(float vel, float acc, DIRECTION dir = POSITIVE);
    void moveHome(float pos, float vel, float acc, float dec, HOMING_MODE mode = DIRECT_HOME);
    void setSoftwareLimit(const AxisSoftwareLimit& limit);
    void setHomingConfig(const AxisHomingConfig& conf);
    bool stopping(float deceleration);
    void resetPosition(float pos);

    uint8_t getAxisId() const;
    bool negativeLimitSwitchActive() const;
    bool positiveLimitSwitchActive() const;

    uint8_t getErrorID() const;

    bool isTargetReached();
    bool isHome() const;
    bool isPowered() const;
    bool inMotion() const;
    bool moveDone() const;


    int32_t getPositionFeedback() const;
    float getVelocityFeedback() const;
    float getLagError() const;
    float getCurrentPosition() const;
    float getCurrentVelocity() const;
    AxisStateId getState() const;
    AxisFlags getFlags();

    MotionProfile::TrapezoidalProfile& getPlanner();
    PulseDriver* getPulseDriver();
    void setState(AxisState* nextState);

private:
    bool handleDriveFault();
    bool isTargetOutOfBounds();
    bool isBrakingBoundaryViolated();
    bool isHardwareLimitTriggered();


    static StandstillState Standstill;
    static DisabledState Disabled;
    static DiscreteMotionState DiscreteMotion;
    static ContinueMotionState ContinueMotion;
    static StoppingState Stopping;
    static HomingState Homing;
    static ErrorState StateError;

    uint8_t axisId{0};
    AxisLimitSwitch* LimitSwitch{};
    AxisHomingConfig HomingConfig{};
    AxisHardware* Hardware;
    AxisStateId stateId{AxisStateId::DISABLED};
    AxisState* currentState;

    MotionProfile::TrapezoidalProfile MotionPlanner;
    PulseDriver pulseDriver;

    float ct{0.0f};
    bool invertDirection{false};
    bool enable{false};
    bool reset{false};
    bool error{false};
    bool resetError{false};
    bool softwareLimitViolation{false};
    uint8_t homingStage{0};
    float homingTimer{0.0f};
    bool isHomingEnabled{false};
    bool move_done{false};



    MoveSettings moveSettings{};
    int8_t backDir{0};
    int32_t positionFeedback{0};
    int32_t velocityFeedback{0};
    float homingStartBuffer = 0.0f;
    ERROR_ID error_id{ERROR_ID::NO_ERROR};
    const float STALL_LAG_THRESHOLD = 30.0f;
};
#endif