
#ifndef CONTROLLOCK_H
#define CONTROLLOCK_H

class ControlLock {
public:
    ControlLock() : saved_(__get_BASEPRI()) {
        __set_BASEPRI_MAX(1u << (8 - __NVIC_PRIO_BITS));
    }
    ~ControlLock() { __set_BASEPRI(saved_); }

    ControlLock(const ControlLock&) = delete;
    ControlLock& operator=(const ControlLock&) = delete;
private:
    uint32_t saved_;
};

#endif //CONTROLLOCK_H
