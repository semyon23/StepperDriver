
#ifndef UNTITLED3_AXESMANAGER_HPP
#define UNTITLED3_AXESMANAGER_HPP

#include "AxisController.hpp"
#include "CommandManager.hpp"

struct TargetDynamics  {
    std::array<float, MAX_NUMBER_DEVICE> positions{};
    std::array<float, MAX_NUMBER_DEVICE> velocities{};
    std::array<float, MAX_NUMBER_DEVICE> accelerations{};
    std::array<float, MAX_NUMBER_DEVICE> deceleration{};
};

class AxesManager {
public:
    AxesManager(AxisControl* x, AxisControl* y, AxisControl* c);
    void recoverAxes(const CommandData *data);
    void powerEnable(const CommandData *data);
    void moveAbsolute(const CommandData *data);
    void moveRelative(const CommandData *data);
    void moveVelocity(const CommandData *data);
    void moveHome(const CommandData *data);
    void haltMove(const CommandData *data);
    void setCurrentPositions(const CommandData *data);

    StatusData getAxesInfo();
    FeedbackData getAxesFeedback();
private:
    template <typename Fn>
    void forEachTarget(const CommandData* data, Fn&& fn);
    bool checkTransaction(uint32_t transaction_id);

    TargetDynamics prepareInterpolatedMotion(uint8_t cmdId, const CommandData *data);
    std::array<AxisControl*, MAX_NUMBER_DEVICE> axes;


    uint8_t current_transaction_id{0};
};




#endif //UNTITLED3_AXISMANAGER_HPP