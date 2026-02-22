#include "mapping.hpp"

int percentToAxis(double value, AxisRange range) {
    if (value < -1.0 || value > 1.0) {
        throw std::invalid_argument("value must be between -1.0 and 1.0");
    }

    const double normalized = (value + 1.0) / 2.0;
    const int mapped = range.minimum + static_cast<int>(std::lround(normalized * range.span()));

    if (mapped < range.minimum) {
        return range.minimum;
    }
    if (mapped > range.maximum) {
        return range.maximum;
    }
    return mapped;
}

int unitToTrigger(double value, int minimum, int maximum) {
    if (value < 0.0 || value > 1.0) {
        throw std::invalid_argument("value must be between 0.0 and 1.0");
    }
    if (minimum >= maximum) {
        throw std::invalid_argument("minimum must be less than maximum");
    }

    const double normalized = value;
    const int mapped = minimum + static_cast<int>(std::lround(normalized * (maximum - minimum)));

    if (mapped < minimum) {
        return minimum;
    }
    if (mapped > maximum) {
        return maximum;
    }
    return mapped;
}
