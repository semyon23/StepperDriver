#include "../include/AxisController.hpp"
#include "../include/AxisHardware.hpp"

DisabledState AxisControl::Disabled;
StandstillState AxisControl::Standstill;
DiscreteMotionState AxisControl::DiscreteMotion;
ContinueMotionState AxisControl::ContinueMotion;
StoppingState AxisControl::Stopping;
ErrorState AxisControl::StateError;
HomingState AxisControl::Homing;


////=========Disable============================
void DisabledState::action(AxisControl* axis) {
    axis->stateId = AxisStateId::DISABLED;
    axis->Hardware->setEnable(true);
}
void DisabledState::setup(AxisControl *axis) {

}

////=========DiscreteMotion============================
void DiscreteMotionState::setup(AxisControl *axis) {

    // float max_vel = axis->SoftwareLimit.velocity;
    // float max_acc = axis->SoftwareLimit.acceleration;
    // float max_dec = axis->SoftwareLimit.deceleration;

    float vel = axis->moveSettings.velocity;
    float acc = axis->moveSettings.acceleration;
    float dec = axis->moveSettings.deceleration;

    float targetPos = axis->moveSettings.distance;
    axis->softwareLimitViolation = false;

    // if (axis->moveSettings.distance > axis->SoftwareLimit.maxPosition) {
    //     axis->softwareLimitViolation = true;
    //     targetPos = axis->SoftwareLimit.maxPosition;
    // } else if (axis->moveSettings.distance < axis->SoftwareLimit.minPosition) {
    //     axis->softwareLimitViolation = true;
    //     targetPos = axis->SoftwareLimit.minPosition;
    // }

    axis->getPlanner().setPosition(targetPos);
    axis->getPlanner().setVelocity(vel);
    axis->getPlanner().setAcceleration(acc);
    axis->getPlanner().setDeceleration(dec);
    axis->getPlanner().setPosition(targetPos);
    // axis->getPlanner().setVelocity(std::max(0.0f, std::min(vel, max_vel)));
    // axis->getPlanner().setAcceleration(std::max(0.0f, std::min(acc, max_acc)));
    // axis->getPlanner().setDeceleration(std::max(0.0f, std::min(dec, max_dec)));

    axis->getPlanner().generate_profile();
}

void DiscreteMotionState::action(AxisControl* axis) {

    if (axis->handleDriveFault()) {
        axis->setState(&AxisControl::StateError);
    } else if (axis->isHardwareLimitTriggered()) {
        axis->getPlanner().hard_stop_profile();
        axis->setState(&AxisControl::StateError);
    } else if (!axis->getPlanner().inMotion() && axis->isTargetReached()) {
        if (axis->isTargetOutOfBounds()) {
            axis->setState(&AxisControl::StateError);
        }else {
            axis->move_done = true;
            axis->setState(&AxisControl::Standstill);
        }
    }else {
        axis->stateId = AxisStateId::DISCRETE_MOTION;
    }
}

////=========ContinueMotion============================
void ContinueMotionState::setup(AxisControl *axis) {
    // auto max_vel = axis->SoftwareLimit.velocity;
    // auto max_acc = axis->SoftwareLimit.acceleration;

    auto vel = axis->moveSettings.velocity;
    auto acc = axis->moveSettings.acceleration;
    auto dir = axis->moveSettings.direction;



    axis->getPlanner().setVelocity(vel);
    axis->getPlanner().setAcceleration(acc);
    // axis->getPlanner().setVelocity(std::clamp(vel, 0.0f, max_vel));
    // axis->getPlanner().setAcceleration(std::clamp(acc, 0.0f, max_acc));
    axis->getPlanner().setDirection(dir);

    axis->getPlanner().continue_profile();
}

void ContinueMotionState::action(AxisControl* axis) {

    if (axis->handleDriveFault()) {
        axis->setState(&AxisControl::StateError);
    }else if (axis->isHardwareLimitTriggered() || axis->isBrakingBoundaryViolated()) {
        //axis->moveSettings.deceleration = axis->SoftwareLimit.deceleration;
        axis->moveSettings.deceleration = axis->moveSettings.acceleration ;
        axis->getPlanner().setDeceleration(axis->moveSettings.deceleration);
        axis->getPlanner().stop_profile();
        axis->setState(&AxisControl::StateError);
    }else {
        axis->stateId = AxisStateId::CONTINUE_MOTION;
    }
}

////=========StoppingState============================
void StoppingState::setup(AxisControl *axis) {
   // auto max_dec = axis->SoftwareLimit.deceleration;
    auto dec = axis->moveSettings.deceleration;
    axis->getPlanner().setDeceleration(dec);
    //axis->getPlanner().setDeceleration(std::clamp(dec, 0.0f, max_dec));
    axis->getPlanner().stop_profile();
}

void StoppingState::action(AxisControl* axis) {
    if (axis->handleDriveFault()) {
        axis->setState(&AxisControl::StateError);
    }else if (!axis->getPlanner().inMotion()) {
        axis->move_done = true;
        axis->setState(&AxisControl::Standstill);
    }else {
        axis->stateId = AxisStateId::STOPPING;
    }
}

////=========HomingState==========================
void HomingState::setup(AxisControl *axis) {
    axis->homingStage = 0;
    axis->homingTimer = 0.0f;
    if (axis->HomingConfig.mode == HOMING_MODE::DIRECT_HOME) {
        return;
    }
    int8_t dir = 1;

    if (axis->HomingConfig.mode == HOMING_MODE::NEGATIVE_LIMIT) {
        dir = -1;
    }
    axis->backDir = -dir;

    axis->getPlanner().setDirection(dir);
    axis->getPlanner().setVelocity(axis->HomingConfig.velocity);
    axis->getPlanner().setAcceleration(axis->HomingConfig.acceleration);
    axis->getPlanner().setDeceleration(axis->HomingConfig.acceleration);
    axis->getPlanner().continue_profile();
}

void HomingState::action(AxisControl *axis) {

    if (axis->handleDriveFault()) {
        axis->setState(&AxisControl::StateError);
    } else {
        axis->stateId = AxisStateId::HOMING;
    }

    auto mode = axis->HomingConfig.mode;

    bool sensor = false;
    if (mode == HOMING_MODE::DIRECT_HOME) {
        axis->resetPosition(axis->HomingConfig.position_offset);
        axis->setState(&AxisControl::Standstill);
        axis->isHomingEnabled = true;
        return;
    } else if (mode == HOMING_MODE::NEGATIVE_LIMIT) {
        sensor = axis->negativeLimitSwitchActive();
    } else if (mode == HOMING_MODE::POSITIVE_LIMIT) {
        sensor = axis->positiveLimitSwitchActive();
    }

    switch (axis->homingStage) {
        case 0:
            if (sensor) {
                axis->getPlanner().setDirection(axis->backDir);

                axis->getPlanner().setVelocity(axis->HomingConfig.crowl_velocity);
                axis->getPlanner().setDeceleration(axis->HomingConfig.acceleration);
                axis->getPlanner().continue_profile();
                axis->homingStage = 1 ;
                axis->homingTimer = 0;
                axis->HomingConfig.timeout = 3;
            }else {
                axis->homingTimer += axis->ct;
                if (axis->homingTimer >= axis->HomingConfig.timeout) {
                    axis->error_id = ERROR_ID::HOMING_TIMEOUT;
                    axis->getPlanner().hard_stop_profile();
                    axis->setState(&AxisControl::StateError);
                }
            }
            break;
        case 1:
            if (!sensor) {
                axis->getPlanner().setDeceleration(axis->HomingConfig.acceleration);
                axis->getPlanner().stop_profile();
                axis->homingStage = 2;
            }else {
                axis->homingTimer += axis->ct;
                if (axis->homingTimer >= axis->HomingConfig.timeout) {
                    axis->error_id = ERROR_ID::HOMING_TIMEOUT;
                    axis->getPlanner().hard_stop_profile();
                    axis->setState(&AxisControl::StateError);
                }
            }
            break;

        case 2:
            if (!axis->getPlanner().inMotion()) {
                axis->resetPosition(axis->HomingConfig.position_offset);
                axis->setState(&AxisControl::Standstill);
                axis->isHomingEnabled = true;
            }
            break;
    }
}
////=========ErrorState===========================
void ErrorState::setup(AxisControl *axis) {
    axis->error = true;

}

void ErrorState::action(AxisControl *axis) {
    axis->stateId = AxisStateId::ERROR;
    if (axis->reset) {
        if (axis->enable) {
            axis->setState(&AxisControl::Standstill);
        }else {
            axis->setState(&AxisControl::Disabled);
        }
    }
}
void ErrorState::cleanup(AxisControl *axis) {
    axis->error = false;
    axis->reset = false;
    axis->error_id = ERROR_ID::NO_ERROR;
}

////=========Standstill==========================
void StandstillState::action(AxisControl *axis) {
    if (axis->handleDriveFault()) {
        axis->setState(&AxisControl::StateError);
    }else {
        axis->stateId = AxisStateId::STANDSTILL;
    }
}

void StandstillState::setup(AxisControl *axis) {
    axis->Hardware->setEnable(false);

}

////=========================================================
AxisControl::AxisControl(uint8_t id, AxisHardware* hw, AxisLimitSwitch* ls, float cycleTime, float tickRate, bool invertDir)
        :axisId(id), Hardware(hw), LimitSwitch(ls), ct(cycleTime), MotionPlanner(cycleTime),
        pulseDriver(hw,tickRate, cycleTime, invertDir),  invertDirection(invertDir) {
    this->currentState = &Disabled;

    // SoftwareLimit.minPosition = -100000.0f;
    // SoftwareLimit.maxPosition = 100000.0f;
    // SoftwareLimit.velocity = 1000000000.0f;
    // SoftwareLimit.acceleration = 1000000000.0f;
    // SoftwareLimit.deceleration = 1000000000.0f;

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

    if (currentState != nextState) {
        if (currentState) {
            currentState->cleanup(this);
        }
        currentState = nextState;
        currentState->setup(this);
    }
    else {
        if (currentState) currentState->setup(this);
    }
}

void AxisControl::update() {
    // if (encoder != nullptr) {
    //     positionFeedback = encoder->updateFeedback();
    //     velocityFeedback = encoder->velocityFeedback(ct);
    // }else {
    //     positionFeedback = static_cast<int32_t>(MotionPlanner.getPosition());
    //     velocityFeedback = MotionPlanner.getVelocity();
    // }
    positionFeedback = static_cast<int32_t>(MotionPlanner.getPosition());
    velocityFeedback = MotionPlanner.getVelocity();

    MotionPlanner.run();

    //pulseDriver.setStepRate(MotionPlanner.getVelocity() * (invertDirection ? -1 : 1));
    pulseDriver.setTargetSteps(MotionPlanner.getPositionSteps());
    currentState->action(this);
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

bool AxisControl::moveRelative(float distance, float vel, float acc, float dec) {
    if (currentState != &Standstill && currentState != &DiscreteMotion) return false;
    move_done = false;
    moveSettings.distance = MotionPlanner.getPosition() + distance;
    moveSettings.velocity = vel;
    moveSettings.acceleration = acc;
    moveSettings.deceleration = dec;
    MotionPlanner.setPosition(moveSettings.distance);
    setState(&DiscreteMotion);
    return true;
}

bool AxisControl::moveAbsolute(float pos, float vel, float acc, float dec) {
    if (currentState != &Standstill && currentState != &DiscreteMotion) return false;
    move_done = false;
    moveSettings.distance = pos;
    moveSettings.velocity = vel;
    moveSettings.acceleration = acc;
    moveSettings.deceleration = dec;
   // MotionPlanner.setPosition(moveSettings.distance);
    setState(&DiscreteMotion);
    return true;
}

bool AxisControl::moveVelocity(float vel, float acc, DIRECTION dir) {
    if (currentState != &Standstill && currentState != &ContinueMotion && currentState != &Stopping) return false;
    move_done = false;
    moveSettings.velocity = vel;
    moveSettings.acceleration = acc;
    moveSettings.direction = dir;

    setState(&ContinueMotion);
    return true;
}

bool AxisControl::moveHome(float pos, float vel, float acc, float dec, HOMING_MODE mode) {
    if (currentState != &Standstill) return false;
    isHomingEnabled = false;

    HomingConfig.position_offset = pos;
    HomingConfig.timeout = 10;
    HomingConfig.mode = mode;
    HomingConfig.velocity = vel;
    HomingConfig.crowl_velocity = vel / 2.0f;
    HomingConfig.acceleration = acc;
    setState(&Homing);
    return true;
}

bool AxisControl::stopping(float deceleration) {
    if (currentState != &ContinueMotion && currentState != &DiscreteMotion && currentState != &Homing) return false;

    moveSettings.deceleration = deceleration;
    setState(&Stopping);
    return true;
}

void AxisControl::resetPosition(float pos) {
    // float min_p = this->SoftwareLimit.minPosition;
    // float max_p = this->SoftwareLimit.maxPosition;
    //float position = std::clamp(pos, min_p, max_p);

    auto _pos = static_cast<int32_t>(pos);

    this->MotionPlanner.setCurrentPosition(pos);
    this->pulseDriver.setCounterPulse(_pos);
   // if (encoder) encoder->setCurrentPosition(_pos);
}

int32_t AxisControl::getPositionFeedback() const {
    return positionFeedback;
}

float AxisControl::getLagError() const{
    int32_t lagError = 0;
    auto position = static_cast<int32_t>(MotionPlanner.getPosition());
    // if (!encoder) {
    //     lagError = position - pulseDriver.getCounterPulse();
    // }else {
    //     lagError = position - getPositionFeedback();
    // }
    lagError = position - pulseDriver.getCounterPulse();
    return lagError;
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


///////////////////////////////////////////////////////////////////////////////////

