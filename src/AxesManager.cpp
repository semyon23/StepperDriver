#include "../include/AxesManager.hpp"

AxesManager::AxesManager(AxisControl *x, AxisControl *y, AxisControl *c): axes{x, y, c}{}

StatusData AxesManager::getAxesInfo() {
    StatusData info{};
    for (size_t i = 0; i < axes.size(); ++i) {
        info.position[i] = axes[i]->getCurrentPosition();
        info.stateId[i] = static_cast<uint8_t>(axes[i]->getState());
        info.isHome[i] = axes[i]->isHome();
        info.isTarget[i] = axes[i]->moveDone();
        info.inMotion[i] = axes[i]->inMotion();
        info.isPowered[i] = axes[i]->isPowered();
        info.errorID[i] = axes[i]->getErrorID();;
    }
    info.transaction_id = this->current_transaction_id;
    return info;
}

FeedbackData AxesManager::getAxesFeedback() {
    FeedbackData data{};
    for (size_t i = 0; i < axes.size(); ++i) {
        data.position[i] = axes[i]->getCurrentPosition();
        data.feedbackPos[i] = static_cast<float>(axes[i]->getPositionFeedback());
        data.lagError[i] = axes[i]->getLagError();
        data.velocity[i] = axes[i]->getCurrentVelocity();
        data.velocityFeedback[i] = axes[i]->getVelocityFeedback();
        data.inMotion[i] = axes[i]->getFlags().inMotion;
        data.inTarget[i] = axes[i]->getFlags().inTarget;
        data.error[i] = axes[i]->getFlags().error;
        data.error_id[i] = axes[i]->getErrorID();
        data.stateId[i] = static_cast<uint8_t>(axes[i]->getState());
        data.negativeLS[i] = axes[i]->negativeLimitSwitchActive();
        data.positiveLS[i] = axes[i]->positiveLimitSwitchActive();
    }
    return data;
}
void AxesManager::powerEnable(const CommandData *data) {
    forEachTarget(data, [&](AxisControl& axis, size_t /*idx*/) {
        if (data->power_enable) {
            axis.enableDriver();
        } else {
            axis.disableDriver();
        }
    });
}

void AxesManager::moveAbsolute(const CommandData *data) {
    bool isGroup = (data->axis_id == AXIS::GROUP);
    auto target = isGroup ? this->prepareInterpolatedMotion(COMMAND::ID::MOVE_ABSOLUTE, data) : TargetDynamics{};

    forEachTarget(data, [&](AxisControl& axis, size_t idx) {
        if (isGroup) {
            axis.moveAbsolute(data->position[idx], target.velocities[idx], target.accelerations[idx], target.deceleration[idx]);
        } else {
            axis.moveAbsolute(data->position[idx], data->velocity, data->acceleration, data->deceleration);
        }
    });
}

void AxesManager::moveRelative(const CommandData *data) {
    bool isGroup = (data->axis_id == AXIS::GROUP);
    auto target = isGroup ? this->prepareInterpolatedMotion(COMMAND::ID::MOVE_RELATIVE, data) : TargetDynamics{};

    forEachTarget(data, [&](AxisControl& axis, size_t idx) {
        if (isGroup) {
            axis.moveRelative(data->position[idx], target.velocities[idx], target.accelerations[idx], target.deceleration[idx]);
        } else {
            axis.moveRelative(data->position[idx], data->velocity, data->acceleration, data->deceleration);
        }
    });
}

void AxesManager::moveVelocity(const CommandData *data) {
    forEachTarget(data, [&](AxisControl &axis, size_t idx) {
        auto dir = static_cast<DIRECTION>(data->direction[idx]);
        axis.moveVelocity(data->velocity, data->acceleration, dir);
    });
}

void AxesManager::moveHome(const CommandData *data) {
    forEachTarget(data, [&](AxisControl &axis, size_t idx) {
        auto mode = static_cast<HOMING_MODE>(data->direction[idx]);
        axis.moveHome(data->position[idx], data->velocity, data->acceleration, data->acceleration, mode);
    });
}

void AxesManager::haltMove(const CommandData *data) {
    forEachTarget(data, [&](AxisControl &axis, size_t /*idx*/) {
        axis.stopping(data->deceleration);
    });
}

void AxesManager::recoverAxes(const CommandData *data) {
    if (!checkTransaction(data->transaction_id)) return;
    for (auto axis : axes) {
        axis->recover();
    }
}

void AxesManager::hardStop(const CommandData *data) {
    if (!checkTransaction(data->transaction_id)) return;
    for (auto axis : axes) {
        axis->emergencyStop();
    }
}


void AxesManager::setCurrentPositions(const CommandData *data) {
    forEachTarget(data, [&](AxisControl& axis, size_t i) {
        axis.resetPosition(data->position[i]);
    });
}

TargetDynamics AxesManager::prepareInterpolatedMotion(uint8_t cmdId, const CommandData *data) {
    TargetDynamics target{};

    float maxDistance = 0.0f;

    for (int i = 0; i < MAX_NUMBER_DEVICE; i++) {
        switch (cmdId) {
            case COMMAND::ID::MOVE_ABSOLUTE: {
                target.positions[i] = std::fabs(data->position[i] - axes[i]->getCurrentPosition());
                break;
            }
            case COMMAND::ID::MOVE_RELATIVE: {
                target.positions[i] = std::fabs(data->position[i]);
                break;
            }
            default:
                target.positions[i] = 0.0f;
                break;
        }

        if (target.positions[i] > maxDistance) {
            maxDistance = target.positions[i];
        }
    }

    if (maxDistance == 0.0f) {
        return target;
    }

    auto kVel = data->velocity / maxDistance;
    auto kAcc = data->acceleration / maxDistance;
    auto kDec = data->deceleration / maxDistance;

    for (int i = 0; i < MAX_NUMBER_DEVICE; i++) {
        target.velocities[i] = kVel * target.positions[i];
        target.accelerations[i] = kAcc * target.positions[i];
        target.deceleration[i] = kDec * target.positions[i];
    }

    return target;
}

template<typename Fn>
void AxesManager::forEachTarget(const CommandData *data, Fn &&fn) {
    if (!checkTransaction(data->transaction_id)) return;
    if (data->axis_id == AXIS::GROUP) {
        for (size_t i = 0; i < axes.size(); ++i) fn(*axes[i], i);
    } else if (data->axis_id < axes.size()) {
        fn(*axes[data->axis_id], data->axis_id);
    }
}

bool AxesManager::checkTransaction(uint32_t transaction_id) {
    if (transaction_id == this->current_transaction_id) {
        return false;
    }
    this->current_transaction_id = transaction_id;
    return true;
}

