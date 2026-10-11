#include "../include/AxisController.hpp"

static bool validDynamics(float vel, float acc, float dec) {
    return std::isfinite(vel) && std::isfinite(acc) && std::isfinite(dec)
        && vel > 0.0f && acc > 0.0f && dec > 0.0f;
}

AxisControl::AxisControl(uint8_t id, AxisHardware* hw, AxisLimitSwitch* ls, float cycleTime, float tickRate, bool invertDir)
        :axisId(id), Hardware(hw), LimitSwitch(ls), ct(cycleTime), MotionPlanner(cycleTime),
        pulseDriver(hw,tickRate, cycleTime, invertDir),  invertDirection(invertDir) {
    this->currentState = &Disabled;

    HomingConfig.mode = HOMING_MODE::DIRECT_HOME;
    HomingConfig.velocity = 5000.0f;
    HomingConfig.crowl_velocity = 500.0f;
    HomingConfig.acceleration = 10000.0f;
    HomingConfig.timeout = 10;

}
uint8_t AxisControl::getAxisId() const {
    return axisId;
}

bool AxisControl::isPowered() const {
    return enable;
}

PulseDriver *AxisControl::getPulseDriver() {
    return &pulseDriver;
}

AxisStateId AxisControl::getState() const{
    return stateId;
}

float AxisControl::getCurrentPosition() const {
    return  this->MotionPlanner.getPosition();
}

float AxisControl::getCurrentVelocity() const {
    return this->MotionPlanner.getVelocity();
}

float AxisControl::getVelocityFeedback() const {
    return velocityFeedback;
}

AxisFlags AxisControl::getFlags() {
    AxisFlags flags{};
    flags.inMotion = this->MotionPlanner.inMotion();
    flags.inTarget = this->isTargetReached();
    flags.error = this->error;
    return flags;
}

MotionProfile::TrapezoidalProfile &AxisControl::getPlanner() {
    return  this->MotionPlanner;
}

void AxisControl::setState(AxisState* nextState) {
    if (!nextState) return;
    if (currentState != nextState) currentState->cleanup(this);
    nextState->setup(this);
    currentState = nextState;
}

void AxisControl::update() {
    if (currentState != &Disabled && currentState != &StateError && handleDriveFault()) {
        setState(&StateError);
    }
    // if (motionStateActive() && isHardwareLimitTriggered()) {   // см. 1.6
    //     MotionPlanner.hard_stop_profile();
    //     setState(&StateError);
    // }
    currentState->action(this);
    MotionPlanner.run();
    pulseDriver.setTargetSteps(MotionPlanner.getPositionSteps());
    positionFeedback = static_cast<int32_t>(MotionPlanner.getPosition());
    velocityFeedback = MotionPlanner.getVelocity();
}

void AxisControl::recover() {
    if (currentState != &StateError) return;
    reset = true;
}

void AxisControl::enableDriver() {
    if (currentState != &Disabled) return;
    enable = true;
    setState(&Standstill);
}

void AxisControl::disableDriver() {
    enable = false;
    if (currentState != &Standstill) return;
    setState(&Disabled);
}

void AxisControl::emergencyStop() {
    MotionPlanner.hard_stop_profile();
    pulseDriver.setCounterPulse(pulseDriver.getCounterPulse());
    enable = false;
    Hardware->setEnable(true);
    error_id = EMERGENCY_STOP;
    setState(&StateError);
}

void AxisControl::moveRelative(float distance, float vel, float acc, float dec) {
    if (currentState != &Standstill && currentState != &DiscreteMotion) return;
    if (!std::isfinite(distance) || !validDynamics(vel, acc, dec)) return;

    move_done = false;
    moveSettings.distance = MotionPlanner.getPosition() + distance;
    moveSettings.velocity = vel;
    moveSettings.acceleration = acc;
    moveSettings.deceleration = dec;
    MotionPlanner.setPosition(moveSettings.distance);
    setState(&DiscreteMotion);
}

void AxisControl::moveAbsolute(float pos, float vel, float acc, float dec) {
    if (currentState != &Standstill && currentState != &DiscreteMotion) return;
    if (!std::isfinite(pos) || !validDynamics(vel, acc, dec)) return;

    move_done = false;
    moveSettings.distance = pos;
    moveSettings.velocity = vel;
    moveSettings.acceleration = acc;
    moveSettings.deceleration = dec;
   // MotionPlanner.setPosition(moveSettings.distance);
    setState(&DiscreteMotion);
}

void AxisControl::moveVelocity(float vel, float acc, DIRECTION dir) {
    if (currentState != &Standstill && currentState != &ContinueMotion && currentState != &Stopping) return;
    if (!std::isfinite(vel) || !std::isfinite(acc)) return;
    if (vel <= 0.0f || acc <= 0.0f) return;
    if (dir != NEGATIVE && dir != POSITIVE) return;

    move_done = false;
    moveSettings.velocity = vel;
    moveSettings.acceleration = acc;
    moveSettings.direction = dir;

    setState(&ContinueMotion);
}

void AxisControl::moveHome(float pos, float vel, float acc, float dec, HOMING_MODE mode) {
    if (currentState != &Standstill) return;
    if (!std::isfinite(pos) || !validDynamics(vel, acc, dec)) return;

    isHomingEnabled = false;
    HomingConfig.position_offset = pos;
    HomingConfig.timeout = 10;
    HomingConfig.mode = mode;
    HomingConfig.velocity = vel;
    HomingConfig.crowl_velocity = vel / 2.0f;
    HomingConfig.acceleration = acc;
    HomingConfig.deceleration = dec;
    setState(&Homing);
}

bool AxisControl::stopping(float deceleration) {
    if (currentState != &ContinueMotion && currentState != &DiscreteMotion && currentState != &Homing) return false;
    if (!std::isfinite(deceleration)) return false;

    moveSettings.deceleration = deceleration;
    setState(&Stopping);
    return true;
}

void AxisControl::resetPosition(float pos) {
    if (!std::isfinite(pos)) return;
    if (MotionPlanner.inMotion()) return;

    MotionPlanner.setCurrentPosition(pos);
    pulseDriver.setCounterPulse(MotionPlanner.getPositionSteps());
}

int32_t AxisControl::getPositionFeedback() const {
    return positionFeedback;
}

float AxisControl::getLagError() const{
    return static_cast<float>(MotionPlanner.getPositionSteps() - pulseDriver.getCounterPulse());
}

bool AxisControl::negativeLimitSwitchActive() const {
    return LimitSwitch->negative;
}

bool AxisControl::positiveLimitSwitchActive() const {
    return LimitSwitch->positive;
}

bool AxisControl::isTargetReached() {
    const int32_t MIN_DISTANCE_THRESHOLD = 1;
    auto target_pos = static_cast<int32_t>(MotionPlanner.getTargetPosition());
    auto current_pos = static_cast<int32_t>(MotionPlanner.getPosition());

    return (std::abs(target_pos  - current_pos) <= MIN_DISTANCE_THRESHOLD);
}

bool AxisControl::isHome() const {
    return this->isHomingEnabled;
}

bool AxisControl::inMotion() const {
    return MotionPlanner.inMotion();
}

bool AxisControl::moveDone() const {
    return this->move_done;
}

void AxisControl::setSoftwareLimit(const AxisSoftwareLimit &limit) {
    //SoftwareLimit = limit;
}
void AxisControl::setHomingConfig(const AxisHomingConfig &conf) {
    HomingConfig = conf;
}

bool AxisControl::handleDriveFault() {
    if (Hardware->faultActive()) {
        getPlanner().hard_stop_profile();
        enable = false;
        Hardware->setEnable(true);
        error_id = ERROR_ID::DRIVE_OVERLOAD;
        return true;
    }
    return false;
}

bool AxisControl::isHardwareLimitTriggered() {
    float dir   = getPlanner().getDirectionSign();
    bool neg_ls = negativeLimitSwitchActive();
    bool pos_ls = positiveLimitSwitchActive();

    if (dir < 0.0f && neg_ls) {
        error_id = ERROR_ID::HARDWARE_NEG_LS_ACTIVE;
        return true;
    }

    if (dir > 0.0f && pos_ls) {
        error_id = ERROR_ID::HARDWARE_POS_LS_ACTIVE;
        return true;
    }
    return false;
}

bool AxisControl::isTargetOutOfBounds() {
    // if (moveSettings.distance > SoftwareLimit.maxPosition) {
    //     error_id = ERROR_ID::SOFTWARE_POS_LS_ACTIVE;
    //     return true;
    // }
    //
    // if (moveSettings.distance < SoftwareLimit.minPosition) {
    //     error_id = ERROR_ID::SOFTWARE_NEG_LS_ACTIVE;
    //     return true;
    // }
    return false;
}

bool AxisControl::isBrakingBoundaryViolated() {
    // float current_pos = MotionPlanner.getPosition();
    // float current_vel = MotionPlanner.getVelocity();
    // float decel       = SoftwareLimit.deceleration;
    //
    //
    // float braking_distance = (current_vel * current_vel) / (2.0f * decel);
    // float discrete_padding = std::fabs(current_vel) * ct;
    // float total_braking_dist = braking_distance + discrete_padding;
    //
    // if (current_vel > 0.0f && MotionPlanner.getDirectionSign() > 0.0f) {
    //     if ((current_pos + total_braking_dist) > SoftwareLimit.maxPosition) {
    //         error_id = ERROR_ID::SOFTWARE_POS_LS_ACTIVE;
    //         return true;
    //     }
    // }
    //
    // if (current_vel < 0.0f && MotionPlanner.getDirectionSign() < 0.0f) {
    //     if ((current_pos - total_braking_dist) < SoftwareLimit.minPosition) {
    //         error_id = ERROR_ID::SOFTWARE_NEG_LS_ACTIVE;
    //         return true;
    //     }
    // }
    return false;
}

uint8_t AxisControl::getErrorID() const {
    return  error_id;
}


