#include "virtual_pad.hpp"

#include <linux/uinput.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>
#include <thread>
#include <vector>

VirtualPad::VirtualPad(AxisRange range) : range_(range) {}

VirtualPad::~VirtualPad() {
    close();
}

void VirtualPad::open() {
    if (fd_ != -1) {
        return;
    }

    fd_ = ::open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (fd_ < 0) {
        throw std::runtime_error("Failed to open /dev/uinput");
    }

    setupDevice();
}

void VirtualPad::close() {
    if (fd_ == -1) {
        return;
    }

    if (ioctl(fd_, UI_DEV_DESTROY) < 0) {
        // Best effort cleanup; ignore errors on teardown.
    }
    ::close(fd_);
    fd_ = -1;
}

int VirtualPad::setRightStickY(double percent) {
    if (fd_ == -1) {
        throw std::runtime_error("Device not opened");
    }

    const int value = percentToAxis(percent, range_);

    sendAbsEvent(ABS_RY, value);

    return value;
}

int VirtualPad::setLeftStickX(double percent) {
    if (fd_ == -1) {
        throw std::runtime_error("Device not opened");
    }

    const int value = percentToAxis(percent, range_);
    sendAbsEvent(ABS_X, value);
    return value;
}

int VirtualPad::setLeftTrigger(double value) {
    if (fd_ == -1) {
        throw std::runtime_error("Device not opened");
    }

    const int mapped = unitToTrigger(value, 0, 255);
    sendAbsEvent(ABS_Z, mapped);
    return mapped;
}

int VirtualPad::setRightTrigger(double value) {
    if (fd_ == -1) {
        throw std::runtime_error("Device not opened");
    }

    const int mapped = unitToTrigger(value, 0, 255);
    sendAbsEvent(ABS_RZ, mapped);
    return mapped;
}

void VirtualPad::setNeutral() {
    if (fd_ == -1) {
        throw std::runtime_error("Device not opened");
    }

    sendAbsEvent(ABS_X, 0);
    sendAbsEvent(ABS_Y, 0);
    sendAbsEvent(ABS_RX, 0);
    sendAbsEvent(ABS_RY, 0);
    sendAbsEvent(ABS_Z, 0);
    sendAbsEvent(ABS_RZ, 0);
    sendAbsEvent(ABS_HAT0X, 0);
    sendAbsEvent(ABS_HAT0Y, 0);
}

void VirtualPad::sendAbsEvent(int code, int value) {
    input_event ev{};
    ev.type = EV_ABS;
    ev.code = static_cast<unsigned short>(code);
    ev.value = value;

    if (::write(fd_, &ev, sizeof(ev)) < 0) {
        throw std::runtime_error("Failed to write ABS event");
    }

    input_event sync{};
    sync.type = EV_SYN;
    sync.code = SYN_REPORT;
    sync.value = 0;
    ::write(fd_, &sync, sizeof(sync));
}

void VirtualPad::setupDevice() {
    if (ioctl(fd_, UI_SET_EVBIT, EV_ABS) < 0) {
        throw std::runtime_error("Failed to set EV_ABS bit");
    }
    if (ioctl(fd_, UI_SET_EVBIT, EV_KEY) < 0) {
        throw std::runtime_error("Failed to set EV_KEY bit");
    }

    const std::vector<int> absCodes = {
        ABS_X,
        ABS_Y,
        ABS_RX,
        ABS_RY,
        ABS_Z,
        ABS_RZ,
        ABS_HAT0X,
        ABS_HAT0Y,
    };

    for (int code : absCodes) {
        if (ioctl(fd_, UI_SET_ABSBIT, code) < 0) {
            throw std::runtime_error("Failed to set ABS bit");
        }
    }

    const std::vector<int> buttonCodes = {
        BTN_GAMEPAD,
        BTN_JOYSTICK,
        BTN_SOUTH,
        BTN_EAST,
        BTN_NORTH,
        BTN_WEST,
        BTN_DPAD_UP,
        BTN_DPAD_DOWN,
        BTN_DPAD_LEFT,
        BTN_DPAD_RIGHT,
        BTN_TL,
        BTN_TR,
        BTN_TL2,
        BTN_TR2,
        BTN_SELECT,
        BTN_START,
        BTN_MODE,
        BTN_THUMBL,
        BTN_THUMBR,
    };

    for (int code : buttonCodes) {
        if (ioctl(fd_, UI_SET_KEYBIT, code) < 0) {
            throw std::runtime_error("Failed to set KEY bit");
        }
    }

    uinput_abs_setup abs_setup{};
    abs_setup.absinfo.minimum = range_.minimum;
    abs_setup.absinfo.maximum = range_.maximum;
    abs_setup.absinfo.fuzz = 0;
    abs_setup.absinfo.flat = 0;
    abs_setup.absinfo.resolution = 0;

    abs_setup.code = ABS_X;
    if (ioctl(fd_, UI_ABS_SETUP, &abs_setup) < 0) {
        throw std::runtime_error("Failed to setup ABS_X");
    }

    abs_setup.code = ABS_Y;
    if (ioctl(fd_, UI_ABS_SETUP, &abs_setup) < 0) {
        throw std::runtime_error("Failed to setup ABS_Y");
    }

    abs_setup.code = ABS_RX;
    if (ioctl(fd_, UI_ABS_SETUP, &abs_setup) < 0) {
        throw std::runtime_error("Failed to setup ABS_RX");
    }

    abs_setup.code = ABS_RY;
    if (ioctl(fd_, UI_ABS_SETUP, &abs_setup) < 0) {
        throw std::runtime_error("Failed to setup ABS_RY");
    }

    uinput_abs_setup trigger_setup{};
    trigger_setup.absinfo.minimum = 0;
    trigger_setup.absinfo.maximum = 255;
    trigger_setup.absinfo.fuzz = 0;
    trigger_setup.absinfo.flat = 0;
    trigger_setup.absinfo.resolution = 0;

    trigger_setup.code = ABS_Z;
    if (ioctl(fd_, UI_ABS_SETUP, &trigger_setup) < 0) {
        throw std::runtime_error("Failed to setup ABS_Z");
    }

    trigger_setup.code = ABS_RZ;
    if (ioctl(fd_, UI_ABS_SETUP, &trigger_setup) < 0) {
        throw std::runtime_error("Failed to setup ABS_RZ");
    }

    uinput_abs_setup hat_setup{};
    hat_setup.absinfo.minimum = -1;
    hat_setup.absinfo.maximum = 1;
    hat_setup.absinfo.fuzz = 0;
    hat_setup.absinfo.flat = 0;
    hat_setup.absinfo.resolution = 0;

    hat_setup.code = ABS_HAT0X;
    if (ioctl(fd_, UI_ABS_SETUP, &hat_setup) < 0) {
        throw std::runtime_error("Failed to setup ABS_HAT0X");
    }

    hat_setup.code = ABS_HAT0Y;
    if (ioctl(fd_, UI_ABS_SETUP, &hat_setup) < 0) {
        throw std::runtime_error("Failed to setup ABS_HAT0Y");
    }

    uinput_setup setup{};
    setup.id.bustype = BUS_USB;
    setup.id.vendor = 0x045e;   // Microsoft
    setup.id.product = 0x028e;  // Xbox 360 Controller
    setup.id.version = 0x0114;
    std::snprintf(setup.name, UINPUT_MAX_NAME_SIZE, "VSAC Controller");

    if (ioctl(fd_, UI_DEV_SETUP, &setup) < 0) {
        throw std::runtime_error("Failed to setup uinput device");
    }
    if (ioctl(fd_, UI_DEV_CREATE) < 0) {
        throw std::runtime_error("Failed to create uinput device");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}
