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
    if (data->axis_id > axes.size()) return;
    if (data->transaction_id == this->current_transaction_id) return;
    this->current_transaction_id = data->transaction_id;

    bool enable = data->power_enable;

    auto apply = [&](AxisControl* axis) {
        enable ? axis->enableDriver() : axis->disableDriver();
    };

    if (data->axis_id == AXIS::GROUP) {
        for (auto* axis : axes) {
            apply(axis);
        }
    } else if (data->axis_id < axes.size()) {
        apply(axes[data->axis_id]);
    }


}

void AxesManager::moveAbsolute(const CommandData *data) {
    if (data->axis_id > axes.size()) return;
    if (data->transaction_id == this->current_transaction_id) return;
    this->current_transaction_id = data->transaction_id;

    switch (data->axis_id) {
        case AXIS::GROUP: {
            auto target = this->prepareInterpolatedMotion(COMMAND::ID::MOVE_ABSOLUTE, data);
            for (size_t i = 0; i < axes.size(); ++i) {
                axes[i]->moveAbsolute(data->position[i], target.velocities[i], target.accelerations[i], target.deceleration[i]);
            }
            break;
        }
        default: {
            axes[data->axis_id]->moveAbsolute(data->position[data->axis_id], data->velocity, data->acceleration, data->deceleration);
            break;
        }
    }
}

void AxesManager::moveRelative(const CommandData *data) {
    if (data->axis_id > axes.size()) return;
    if (data->transaction_id == this->current_transaction_id) return;
    this->current_transaction_id = data->transaction_id;
    switch (data->axis_id) {
        case AXIS::GROUP: {
            auto target = this->prepareInterpolatedMotion(COMMAND::ID::MOVE_RELATIVE, data);
            for (size_t i = 0; i < axes.size(); ++i) {
                axes[i]->moveRelative(data->position[i], target.velocities[i], target.accelerations[i], target.deceleration[i]);
            }
            break;
        }
        default: {
            axes[data->axis_id]->moveRelative(data->position[data->axis_id], data->velocity, data->acceleration, data->deceleration);
            break;
        }
    }
}

void AxesManager::moveVelocity(const CommandData *data) {
    if (data->axis_id > axes.size()) return;
    if (data->transaction_id == this->current_transaction_id) return;
    this->current_transaction_id = data->transaction_id;
    switch (data->axis_id) {
        case AXIS::GROUP: {
            for (size_t i = 0; i < axes.size(); ++i) {
                auto dir = static_cast<DIRECTION>(data->direction[i]);
                axes[i]->moveVelocity(data->velocity,data->acceleration, dir);
            }
            break;
        }
        default: {
            auto dir = static_cast<DIRECTION>(data->direction[data->axis_id]);
            axes[data->axis_id]->moveVelocity(data->velocity,data->acceleration, dir);
            break;
        }
    }
}


void AxesManager::moveHome(const CommandData *data) {
    if (data->axis_id > axes.size()) return;
    if (data->transaction_id == this->current_transaction_id) return;
    this->current_transaction_id = data->transaction_id;
    switch (data->axis_id) {
        case AXIS::GROUP: {
            for (size_t i = 0; i < axes.size(); ++i) {
                auto mode = static_cast<HOMING_MODE>(data->direction[i]);
                axes[i]->moveHome(data->position[i],data->velocity, data->acceleration, data->deceleration,  mode);
            }
            break;
        }
        default: {
            auto mode = static_cast<HOMING_MODE>(data->direction[data->axis_id]);
            axes[data->axis_id]->moveHome(data->position[data->axis_id],data->velocity, data->acceleration, data->deceleration, mode);
            break;
        }
    }
}

void AxesManager::haltMove(const CommandData *data) {
    if (data->axis_id > axes.size()) return;
    if (data->transaction_id == this->current_transaction_id) return;
    this->current_transaction_id = data->transaction_id;
    switch (data->axis_id) {
        case AXIS::GROUP: {
            for (auto axis : axes) {
                axis->stopping(data->deceleration);
            }
            break;
        }
        default: {
            axes[data->axis_id]->stopping(data->deceleration);
            break;
        }
    }
}

void AxesManager::recoverAxes(const CommandData *data) {
    if (data->transaction_id == this->current_transaction_id) return;
    this->current_transaction_id = data->transaction_id;
    for (auto axis : axes) {
        axis->recover();
    }
}

void AxesManager::setCurrentPositions(const CommandData *data) {

    switch (data->axis_id) {
        case AXIS::GROUP: {
            for (size_t i = 0; i < axes.size(); ++i) {
                axes[i]->resetPosition(data->position[i]);
            }
            break;
        }
        default: {
            axes[data->axis_id]->resetPosition(data->position[data->axis_id]);
            break;
        }
    }
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

    // 3. Распределение скоростей и ускорений по осям
    for (int i = 0; i < MAX_NUMBER_DEVICE; i++) {
        target.velocities[i] = kVel * target.positions[i];
        target.accelerations[i] = kAcc * target.positions[i];
        target.deceleration[i] = kDec * target.positions[i];
    }

    return target;
}
// TargetDynamics AxesManager::prepareInterpolatedMotion(uint8_t cmdId, const CommandData *data) {
//     TargetDynamics target{};
//
//     for (int i = 0; i < MAX_NUMBER_DEVICE; i++) {
//         switch (cmdId) {
//             case COMMAND::ID::MOVE_ABSOLUTE: {
//                 target.positions[i] = std::fabs(data->position[i] - axes[i]->getCurrentPosition());
//                 break;
//             }
//             case COMMAND::ID::MOVE_RELATIVE: {
//                 target.positions[i] = std::fabs(data->position[i]);
//                 break;
//             }
//             default:
//                 target.positions[i] = 0.0f;
//                 break;
//         }
//     }
//     auto _distance = std::sqrt((target.positions[0]*target.positions[0]) +
//                                        (target.positions[1]*target.positions[1]) +
//                                        (target.positions[2]*target.positions[2]));
//     if (_distance == 0) return target;
//
//     auto kVel = data->velocity / _distance;
//     auto kAcc = data->acceleration / data->velocity;
//     auto kDec = data->deceleration / data->velocity;
//
//     for (int i = 0; i < MAX_NUMBER_DEVICE; i++) {
//         target.velocities[i] = kVel * target.positions[i];
//         target.accelerations[i] = kAcc * target.velocities[i];
//         target.deceleration[i] = kDec * target.velocities[i];
//     }
//
//     return target;
// }

void AxesManager::setAxisSoftwareLimit(const AxisSoftwareLimit *data) {
    if (data == nullptr) return;

    for (size_t i = 0; i < 3; ++i) {
        AxisSoftwareLimit limit = data[i];
        axes[i]->setSoftwareLimit(limit);
    }
}

void AxesManager::setHomingConfig(const AxisHomingConfig *data) {
    if (data == nullptr) return;

    for (size_t i = 0; i < 3; ++i) {
        AxisHomingConfig conf = data[i];
        axes[i]->setHomingConfig(conf);
    }
}

