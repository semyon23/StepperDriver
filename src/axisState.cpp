#include "../include/AxisController.hpp"

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

