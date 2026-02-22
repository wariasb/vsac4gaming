#pragma once

#include <cmath>
#include <stdexcept>

struct AxisRange {
    int minimum = -32768;
    int maximum = 32767;

    int span() const {
        return maximum - minimum;
    }
};

int percentToAxis(double value, AxisRange range = AxisRange());
int unitToTrigger(double value, int minimum = 0, int maximum = 255);
