#pragma once

#include <cmath>
#include <stdexcept>

struct AxisRange {
    int minimum = -24576;
    int maximum = 24575;

    int span() const {
        return maximum - minimum;
    }
};

int percentToAxis(double value, AxisRange range = AxisRange());
int unitToTrigger(double value, int minimum = 0, int maximum = 255);
