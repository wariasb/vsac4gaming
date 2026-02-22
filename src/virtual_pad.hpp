#pragma once

#include "mapping.hpp"

class VirtualPad {
public:
    explicit VirtualPad(AxisRange range = AxisRange());
    ~VirtualPad();

    void open();
    void close();
    void setNeutral();
    int setLeftStickX(double percent);
    int setRightStickY(double percent);
    int setLeftTrigger(double value);
    int setRightTrigger(double value);

private:
    int fd_ = -1;
    AxisRange range_;

    void sendAbsEvent(int code, int value);

    void setupDevice();
};
