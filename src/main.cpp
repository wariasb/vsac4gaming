#include "pine.h"
#include "virtual_pad.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include "vsac4gaming/msg/vehicle_state.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <algorithm>

namespace {
constexpr double kAxisMin = -1.0;
constexpr double kAxisMax = 1.0;
constexpr double kTriggerMin = 0.0;
constexpr double kTriggerMax = 1.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr float kRadToDeg = 57.2957795f;

constexpr double clamp(double value, double minValue, double maxValue) {
    return value < minValue ? minValue : (value > maxValue ? maxValue : value);
}

double normalizeAnglePi(double angle) {
    if (!std::isfinite(angle)) {
        return 0.0;
    }

    angle = std::fmod(angle, kTwoPi);
    if (angle > kPi) {
        angle -= kTwoPi;
    } else if (angle < -kPi) {
        angle += kTwoPi;
    }
    return angle;
}

struct PineTelemetry {
    float pos_x = 0.0f;
    float pos_y = 0.0f;
    float pos_z = 0.0f;
    float speed = 0.0f;
    float throttle_feedback = 0.0f;
    float brake_feedback = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float steering_command = 0.0f;
    float steering_angle_rad = 0.0f;
    float steering_angle_deg = 0.0f;
};

struct PineAddresses {
    uint32_t pos_x = 0;
    uint32_t pos_y = 0;
    uint32_t pos_z = 0;
    uint32_t speed = 0;
    uint32_t throttle_feedback = 0;
    uint32_t brake_feedback = 0;
    uint32_t yaw = 0;
    uint32_t pitch = 0;
    uint32_t roll = 0;
    uint32_t steering_feedback = 0;
    uint32_t steering_angle_rad = 0;
};

std::string trim(const std::string& value) {
    const auto start = value.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
        return "";
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(start, end - start + 1);
}

std::optional<uint32_t> parseAddress(const std::string& value) {
    try {
        size_t idx = 0;
        const unsigned long parsed = std::stoul(value, &idx, 0);
        if (idx != value.size()) {
            return std::nullopt;
        }
        return static_cast<uint32_t>(parsed);
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

PineAddresses loadPineAddresses(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("Failed to open " + path);
    }

    PineAddresses addresses{};
    bool has_pos_x = false;
    bool has_pos_y = false;
    bool has_pos_z = false;
    bool has_speed = false;
    bool has_throttle = false;
    bool has_brake = false;
    bool has_yaw = false;
    bool has_pitch = false;
    bool has_roll = false;
    bool has_steering_feedback = false;
    bool has_steering_angle = false;

    std::string line;
    while (std::getline(file, line)) {
        const std::string cleaned = trim(line);
        if (cleaned.empty() || cleaned[0] == '#' || cleaned[0] == ';') {
            continue;
        }

        const auto equals = cleaned.find('=');
        if (equals == std::string::npos) {
            continue;
        }

        const std::string key = trim(cleaned.substr(0, equals));
        const std::string value = trim(cleaned.substr(equals + 1));
        const auto parsed = parseAddress(value);
        if (!parsed.has_value()) {
            throw std::runtime_error("Invalid address for key: " + key);
        }

        if (key == "pos_x") {
            addresses.pos_x = parsed.value();
            has_pos_x = true;
        } else if (key == "pos_y") {
            addresses.pos_y = parsed.value();
            has_pos_y = true;
        } else if (key == "pos_z") {
            addresses.pos_z = parsed.value();
            has_pos_z = true;
        } else if (key == "speed") {
            addresses.speed = parsed.value();
            has_speed = true;
        } else if (key == "throttle_feedback") {
            addresses.throttle_feedback = parsed.value();
            has_throttle = true;
        } else if (key == "brake_feedback") {
            addresses.brake_feedback = parsed.value();
            has_brake = true;
        } else if (key == "yaw") {
            addresses.yaw = parsed.value();
            has_yaw = true;
        } else if (key == "pitch") {
            addresses.pitch = parsed.value();
            has_pitch = true;
        } else if (key == "roll") {
            addresses.roll = parsed.value();
            has_roll = true;
        } else if (key == "steering_feedback") {
            addresses.steering_feedback = parsed.value();
            has_steering_feedback = true;
        } else if (key == "steering_angle_rad") {
            addresses.steering_angle_rad = parsed.value();
            has_steering_angle = true;
        }
    }

    if (!has_pos_x || !has_pos_y || !has_pos_z || !has_speed || !has_throttle || !has_brake ||
        !has_yaw || !has_pitch || !has_roll || !has_steering_feedback || !has_steering_angle) {
        throw std::runtime_error("Missing required memory addresses in " + path);
    }

    return addresses;
}

std::string findConfigPath() {
    namespace fs = std::filesystem;
    const fs::path cwd = fs::current_path();
    const fs::path config_name("config/memory_address.ini");
    const fs::path candidates[] = {
        cwd / config_name,
        cwd.parent_path() / config_name,
        cwd.parent_path().parent_path() / config_name,
    };

    for (const auto& candidate : candidates) {
        if (fs::exists(candidate)) {
            return candidate.string();
        }
    }

    throw std::runtime_error("Could not locate config/memory_address.ini");
}

bool readPineTelemetry(PINE::PCSX2& ipc, const PineAddresses& addresses, PineTelemetry& out) {
    try {
        auto readFloat = [&ipc](uint32_t address) -> float {
            const uint32_t raw = ipc.Read<uint32_t>(address);
            float value = 0.0f;
            std::memcpy(&value, &raw, sizeof(value));
            return value;
        };

        out.speed = readFloat(addresses.speed);
        out.pos_x = readFloat(addresses.pos_x);
        out.pos_y = readFloat(addresses.pos_y);
        out.pos_z = readFloat(addresses.pos_z);
        out.throttle_feedback = readFloat(addresses.throttle_feedback);
        out.brake_feedback = readFloat(addresses.brake_feedback);
        out.yaw = readFloat(addresses.yaw);
        out.pitch = readFloat(addresses.pitch);
        out.roll = readFloat(addresses.roll);
        const float steering_raw = readFloat(addresses.steering_feedback);
        const float steering_angle_rad = readFloat(addresses.steering_angle_rad);
        out.steering_command = (steering_raw - 0.5f) * 200.0f;
        out.steering_angle_rad = steering_angle_rad;
        out.steering_angle_deg = steering_angle_rad * kRadToDeg;
        return true;
    } catch (PINE::Shared::IPCStatus status) {
        std::cerr << "PINE read error: " << static_cast<unsigned>(status) << std::endl;
    } catch (const std::exception& ex) {
        std::cerr << "PINE exception: " << ex.what() << std::endl;
    }
    return false;
}

class VcuRosNode : public rclcpp::Node {
public:
    VcuRosNode() : Node("vcu_controller") {
        const std::string config_path = findConfigPath();
        addresses_ = loadPineAddresses(config_path);
        RCLCPP_INFO(get_logger(), "Loaded memory addresses from %s", config_path.c_str());

        declare_parameter("steering_topic", std::string("/control/steering_command"));
        declare_parameter("throttle_topic", std::string("/control/throttle_command"));
        declare_parameter("brake_topic", std::string("/control/brake_command"));
        declare_parameter("publish_rate_hz", 25.0);
        declare_parameter("throttle_scale", 1.0);
        declare_parameter("brake_scale", 1.0);
        declare_parameter("pos_x_topic", std::string("/localization/pos/x"));
        declare_parameter("pos_y_topic", std::string("/localization/pos/y"));
        declare_parameter("pos_z_topic", std::string("/localization/pos/z"));
        declare_parameter("speed_topic", std::string("/localization/vel/x"));
        declare_parameter("throttle_feedback_topic", std::string("/vi/throttle_feedback"));
        declare_parameter("brake_feedback_topic", std::string("/vi/brake_feedback"));
        declare_parameter("accel_topic", std::string("/localization/acc/x"));
        declare_parameter("roll_topic", std::string("/localization/orient/roll"));
        declare_parameter("pitch_topic", std::string("/localization/orient/pitch"));
        declare_parameter("yaw_topic", std::string("/localization/orient/yaw"));
        declare_parameter("steering_cmd_topic", std::string("/vi/steering_feedback"));
        declare_parameter("steering_angle_topic", std::string("/vi/steering_angle_rad"));
        declare_parameter("steering_angle_deg_topic", std::string("/vi/steering_angle_deg"));
        declare_parameter("vehicle_state_topic", std::string("/vehicle_state"));

        steering_topic_ = get_parameter("steering_topic").as_string();
        throttle_topic_ = get_parameter("throttle_topic").as_string();
        brake_topic_ = get_parameter("brake_topic").as_string();
        publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
        throttle_scale_ = get_parameter("throttle_scale").as_double();
        brake_scale_ = get_parameter("brake_scale").as_double();
        pos_x_topic_ = get_parameter("pos_x_topic").as_string();
        pos_y_topic_ = get_parameter("pos_y_topic").as_string();
        pos_z_topic_ = get_parameter("pos_z_topic").as_string();
        speed_topic_ = get_parameter("speed_topic").as_string();
        throttle_feedback_topic_ = get_parameter("throttle_feedback_topic").as_string();
        brake_feedback_topic_ = get_parameter("brake_feedback_topic").as_string();
        accel_topic_ = get_parameter("accel_topic").as_string();
        roll_topic_ = get_parameter("roll_topic").as_string();
        pitch_topic_ = get_parameter("pitch_topic").as_string();
        yaw_topic_ = get_parameter("yaw_topic").as_string();
        steering_cmd_topic_ = get_parameter("steering_cmd_topic").as_string();
        steering_angle_topic_ = get_parameter("steering_angle_topic").as_string();
        steering_angle_deg_topic_ = get_parameter("steering_angle_deg_topic").as_string();
        vehicle_state_topic_ = get_parameter("vehicle_state_topic").as_string();

        pad_.open();
        pad_.setNeutral();

        steering_sub_ = create_subscription<std_msgs::msg::Float32>(
            steering_topic_, 10,
            [this](const std_msgs::msg::Float32& msg) {
                const double value = clamp(msg.data, kAxisMin, kAxisMax);
                steering_.store(value);
                RCLCPP_INFO(get_logger(), "Steering: %.3f", value);
                std::cout << "Steering: " << value << std::endl;
            });

        throttle_sub_ = create_subscription<std_msgs::msg::Float32>(
            throttle_topic_, 10,
            [this](const std_msgs::msg::Float32& msg) {
                const double value = clamp(msg.data, kTriggerMin, kTriggerMax);
                throttle_.store(value);
                RCLCPP_INFO(get_logger(), "Throttle: %.3f", value);
                std::cout << "Throttle: " << value << std::endl;
            });

        brake_sub_ = create_subscription<std_msgs::msg::Float32>(
            brake_topic_, 10,
            [this](const std_msgs::msg::Float32& msg) {
                const double value = clamp(msg.data, kTriggerMin, kTriggerMax);
                brake_.store(value);
                RCLCPP_INFO(get_logger(), "Brake: %.3f", value);
                std::cout << "Brake: " << value << std::endl;
            });

        pos_x_pub_ = create_publisher<std_msgs::msg::Float32>(pos_x_topic_, 10);
        pos_y_pub_ = create_publisher<std_msgs::msg::Float32>(pos_y_topic_, 10);
        pos_z_pub_ = create_publisher<std_msgs::msg::Float32>(pos_z_topic_, 10);
        speed_pub_ = create_publisher<std_msgs::msg::Float32>(speed_topic_, 10);
        throttle_feedback_pub_ = create_publisher<std_msgs::msg::Float32>(throttle_feedback_topic_, 10);
        brake_feedback_pub_ = create_publisher<std_msgs::msg::Float32>(brake_feedback_topic_, 10);
        accel_pub_ = create_publisher<std_msgs::msg::Float32>(accel_topic_, 10);
        roll_pub_ = create_publisher<std_msgs::msg::Float32>(roll_topic_, 10);
        pitch_pub_ = create_publisher<std_msgs::msg::Float32>(pitch_topic_, 10);
        yaw_pub_ = create_publisher<std_msgs::msg::Float32>(yaw_topic_, 10);
        steering_cmd_pub_ = create_publisher<std_msgs::msg::Float32>(steering_cmd_topic_, 10);
        steering_angle_pub_ = create_publisher<std_msgs::msg::Float32>(steering_angle_topic_, 10);
        steering_angle_deg_pub_ =
            create_publisher<std_msgs::msg::Float32>(steering_angle_deg_topic_, 10);
        vehicle_state_pub_ =
            create_publisher<vsac4gaming::msg::VehicleState>(vehicle_state_topic_, 10);

        const double rate = publish_rate_hz_ > 0.0 ? publish_rate_hz_ : 60.0;
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            [this]() { publishInputs(); });

        RCLCPP_INFO(get_logger(),
                    "Listening on %s (steering), %s (throttle), %s (brake) at %.1f Hz",
                    steering_topic_.c_str(),
                    throttle_topic_.c_str(),
                    brake_topic_.c_str(),
                    rate);
    }

    ~VcuRosNode() override {
        pad_.close();
    }

private:
    bool shouldReadPine() {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_pine_read_ >= pine_read_interval_) {
            last_pine_read_ = now;
            return true;
        }
        return false;
    }

    void publishPineTelemetry() {
        PineTelemetry telemetry;
        if (!readPineTelemetry(ipc_, addresses_, telemetry)) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        float accel = 0.0f;
        float orient_vel_x = 0.0f;
        float orient_vel_y = 0.0f;
        float orient_vel_z = 0.0f;
        if (has_last_speed_) {
            const std::chrono::duration<float> dt = now - last_speed_time_;
            if (dt.count() > 0.0f) {
                accel = (telemetry.speed - last_speed_) / dt.count();
            }
        }
        if (has_last_orient_) {
            const std::chrono::duration<float> dt = now - last_orient_time_;
            if (dt.count() > 0.0f) {
                const float d_roll = static_cast<float>(normalizeAnglePi(telemetry.roll - last_roll_));
                const float d_pitch = static_cast<float>(normalizeAnglePi(telemetry.pitch - last_pitch_));
                const float d_yaw = static_cast<float>(normalizeAnglePi(telemetry.yaw - last_yaw_));
                orient_vel_x = d_roll / dt.count();
                orient_vel_y = d_pitch / dt.count();
                orient_vel_z = d_yaw / dt.count();
            }
        }
        last_speed_ = telemetry.speed;
        last_speed_time_ = now;
        has_last_speed_ = true;
        last_roll_ = telemetry.roll;
        last_pitch_ = telemetry.pitch;
        last_yaw_ = telemetry.yaw;
        last_orient_time_ = now;
        has_last_orient_ = true;

        std_msgs::msg::Float32 msg;
        msg.data = telemetry.pos_x;
        pos_x_pub_->publish(msg);
        msg.data = telemetry.pos_y;
        pos_y_pub_->publish(msg);
        msg.data = telemetry.pos_z;
        pos_z_pub_->publish(msg);
        msg.data = telemetry.speed;
        speed_pub_->publish(msg);
        msg.data = telemetry.throttle_feedback;
        throttle_feedback_pub_->publish(msg);
        msg.data = telemetry.brake_feedback;
        brake_feedback_pub_->publish(msg);
        msg.data = accel;
        accel_pub_->publish(msg);
        msg.data = telemetry.roll;
        roll_pub_->publish(msg);
        msg.data = telemetry.pitch;
        pitch_pub_->publish(msg);
        msg.data = normalizeAnglePi(telemetry.yaw);
        yaw_pub_->publish(msg);
        msg.data = telemetry.steering_command;
        steering_cmd_pub_->publish(msg);
        msg.data = telemetry.steering_angle_rad;
        steering_angle_pub_->publish(msg);
        msg.data = telemetry.steering_angle_deg;
        steering_angle_deg_pub_->publish(msg);

        vsac4gaming::msg::VehicleState state_msg;
        state_msg.pos.x = telemetry.pos_x;
        state_msg.pos.y = telemetry.pos_y;
        state_msg.pos.z = telemetry.pos_z;
        state_msg.vel.x = telemetry.speed;
        state_msg.vel.y = 0.0;
        state_msg.vel.z = 0.0;
        state_msg.linear_acc.x = accel;
        state_msg.linear_acc.y = 0.0;
        state_msg.linear_acc.z = 0.0;
        state_msg.orient.x = telemetry.roll;
        state_msg.orient.y = telemetry.pitch;
        state_msg.orient.z = normalizeAnglePi(telemetry.yaw);
        state_msg.orient_vel.x = orient_vel_x;
        state_msg.orient_vel.y = orient_vel_y;
        state_msg.orient_vel.z = orient_vel_z;
        vehicle_state_pub_->publish(state_msg);
    }

    /**
     * Maps Target Y [-1.0, 1.0] to Required X.
     * Derived from high-density calibration data.
     */
    double getRequiredX_Final(double targetY) {
        // index = |targetY| * 1000. Value = Required X.
        static const double x_lut[1001] = {
            0.000000, 0.039173, 0.078346, 0.117518, 0.156691, 0.195864, 0.235036, 0.274209, 0.305940, 0.307010,
            0.308079, 0.309149, 0.310219, 0.311288, 0.312358, 0.313428, 0.314498, 0.315367, 0.316823, 0.318279,
            0.319736, 0.321192, 0.322649, 0.324105, 0.325633, 0.327170, 0.328708, 0.330245, 0.331783, 0.333321,
            0.334858, 0.337213, 0.339665, 0.342118, 0.344571, 0.345680, 0.346853, 0.348025, 0.349198, 0.350371,
            0.351543, 0.352716, 0.353889, 0.355061, 0.356234, 0.357335, 0.358284, 0.359489, 0.360694, 0.361900,
            0.363105, 0.364310, 0.365516, 0.366721, 0.367926, 0.369131, 0.370337, 0.371542, 0.372747, 0.373953,
            0.375158, 0.376363, 0.377569, 0.378774, 0.379979, 0.381185, 0.382390, 0.383595, 0.384801, 0.386006,
            0.387211, 0.387869, 0.389148, 0.390428, 0.391708, 0.392988, 0.394268, 0.395276, 0.396599, 0.397921,
            0.399243, 0.400508, 0.401918, 0.403328, 0.404738, 0.405527, 0.406795, 0.408063, 0.409331, 0.410599,
            0.411867, 0.413135, 0.414403, 0.415444, 0.416896, 0.418347, 0.419798, 0.421250, 0.422701, 0.424153,
            0.425114, 0.426541, 0.427968, 0.429395, 0.430822, 0.432249, 0.433433, 0.434720, 0.436006, 0.437292,
            0.438579, 0.439865, 0.441151, 0.442438, 0.443724, 0.445010, 0.446297, 0.447583, 0.448869, 0.450156,
            0.451442, 0.452728, 0.454015, 0.455301, 0.456587, 0.457874, 0.459160, 0.460446, 0.461733, 0.463019,
            0.464305, 0.465592, 0.466878, 0.468164, 0.469451, 0.470737, 0.472023, 0.473310, 0.474596, 0.475882,
            0.477169, 0.478455, 0.479741, 0.481028, 0.481310, 0.482436, 0.483562, 0.484687, 0.485813, 0.486938,
            0.488064, 0.489189, 0.490315, 0.491440, 0.492566, 0.493691, 0.494817, 0.495942, 0.497068, 0.498193,
            0.499319, 0.500444, 0.501570, 0.502695, 0.503821, 0.504946, 0.506072, 0.507197, 0.508323, 0.509448,
            0.510574, 0.511699, 0.512825, 0.513950, 0.515076, 0.516201, 0.517327, 0.518452, 0.519578, 0.520703,
            0.521380, 0.522209, 0.523038, 0.523867, 0.524697, 0.525526, 0.526355, 0.527184, 0.528014, 0.528843,
            0.529672, 0.530501, 0.531331, 0.532160, 0.532989, 0.533818, 0.534213, 0.535090, 0.535967, 0.536845,
            0.537722, 0.538600, 0.539477, 0.540354, 0.541232, 0.542109, 0.542987, 0.543864, 0.544741, 0.545619,
            0.546496, 0.547374, 0.548251, 0.549128, 0.550006, 0.550883, 0.551761, 0.552638, 0.553515, 0.554393,
            0.555132, 0.556276, 0.557419, 0.558563, 0.559706, 0.560850, 0.561993, 0.563137, 0.564280, 0.565424,
            0.566567, 0.567711, 0.568854, 0.569998, 0.571141, 0.572285, 0.573428, 0.574572, 0.575715, 0.576859,
            0.577607, 0.578676, 0.579745, 0.580814, 0.581882, 0.582951, 0.584020, 0.585089, 0.586158, 0.587227,
            0.588295, 0.589364, 0.590300, 0.591244, 0.592187, 0.593131, 0.594075, 0.595018, 0.595962, 0.596906,
            0.597849, 0.598793, 0.599737, 0.600680, 0.601624, 0.602568, 0.603511, 0.604455, 0.605399, 0.606342,
            0.607286, 0.608230, 0.609173, 0.610117, 0.611061, 0.612004, 0.612948, 0.613892, 0.614835, 0.615779,
            0.616340, 0.617112, 0.617883, 0.618655, 0.619427, 0.620199, 0.620971, 0.621742, 0.622514, 0.623286,
            0.624058, 0.624830, 0.625488, 0.626442, 0.627397, 0.628352, 0.629307, 0.630261, 0.631216, 0.632171,
            0.633126, 0.634080, 0.635035, 0.635990, 0.636945, 0.637900, 0.638854, 0.639809, 0.640764, 0.641719,
            0.642673, 0.643628, 0.644583, 0.645538, 0.646492, 0.647447, 0.648402, 0.649357, 0.649930, 0.650893,
            0.651857, 0.652820, 0.653784, 0.654747, 0.655710, 0.656674, 0.657637, 0.658601, 0.659564, 0.660527,
            0.661491, 0.662454, 0.663418, 0.664381, 0.665345, 0.666308, 0.667271, 0.668235, 0.669198, 0.670162,
            0.671125, 0.672088, 0.673052, 0.674015, 0.674979, 0.675942, 0.676905, 0.677869, 0.678832, 0.679796,
            0.680759, 0.681722, 0.682686, 0.683649, 0.684613, 0.684990, 0.685987, 0.686984, 0.687982, 0.688979,
            0.689976, 0.690973, 0.691970, 0.692420, 0.693427, 0.694435, 0.695442, 0.696449, 0.697457, 0.698464,
            0.699471, 0.700479, 0.701486, 0.702494, 0.703501, 0.704508, 0.705516, 0.706523, 0.707530, 0.708538,
            0.709545, 0.710552, 0.711560, 0.712567, 0.713440, 0.714454, 0.715468, 0.716482, 0.717496, 0.718510,
            0.719524, 0.720538, 0.721552, 0.722566, 0.723580, 0.724594, 0.725608, 0.726622, 0.727636, 0.728650,
            0.729664, 0.730678, 0.731692, 0.732706, 0.733720, 0.734734, 0.735748, 0.736762, 0.737776, 0.738790,
            0.739804, 0.740818, 0.741832, 0.742846, 0.743860, 0.744874, 0.745888, 0.746902, 0.747916, 0.748930,
            0.749944, 0.750958, 0.751972, 0.752986, 0.754000, 0.755014, 0.756028, 0.757042, 0.758056, 0.759070,
            0.760084, 0.761098, 0.762112, 0.763126, 0.764140, 0.765154, 0.766168, 0.767182, 0.768221, 0.769260,
            0.770299, 0.771338, 0.772377, 0.773416, 0.774455, 0.775494, 0.776533, 0.777572, 0.778611, 0.779650,
            0.780689, 0.781728, 0.782210, 0.783141, 0.784071, 0.785002, 0.785933, 0.786864, 0.787794, 0.788725,
            0.789656, 0.790587, 0.791517, 0.792448, 0.793379, 0.794310, 0.795241, 0.796171, 0.797102, 0.798033,
            0.798964, 0.799894, 0.800825, 0.801756, 0.802687, 0.803617, 0.804548, 0.805479, 0.806410, 0.807340,
            0.808271, 0.809202, 0.810133, 0.811064, 0.811994, 0.812925, 0.813856, 0.814787, 0.815717, 0.816648,
            0.817579, 0.818510, 0.819441, 0.820371, 0.821302, 0.822233, 0.823164, 0.824094, 0.825025, 0.825956,
            0.826887, 0.827818, 0.828748, 0.829679, 0.830610, 0.831541, 0.832471, 0.833402, 0.834333, 0.835264,
            0.836195, 0.837125, 0.838056, 0.838987, 0.839918, 0.840848, 0.841779, 0.842710, 0.843641, 0.844572,
            0.845502, 0.846433, 0.847364, 0.848295, 0.849225, 0.850156, 0.851087, 0.852018, 0.852949, 0.853879,
            0.853940, 0.854611, 0.855282, 0.855953, 0.856623, 0.857294, 0.857965, 0.858636, 0.859306, 0.859977,
            0.860648, 0.861319, 0.861989, 0.862660, 0.863331, 0.864002, 0.864672, 0.865343, 0.866014, 0.866685,
            0.867355, 0.868026, 0.868697, 0.869368, 0.870038, 0.870709, 0.871380, 0.872051, 0.872721, 0.873392,
            0.874063, 0.874734, 0.875404, 0.876075, 0.876746, 0.877417, 0.878087, 0.878758, 0.879429, 0.880100,
            0.880770, 0.881441, 0.882112, 0.882783, 0.883453, 0.884124, 0.884795, 0.885466, 0.886136, 0.886807,
            0.887478, 0.888149, 0.888819, 0.889490, 0.890161, 0.890832, 0.891502, 0.892173, 0.892844, 0.892992,
            0.893666, 0.894340, 0.895014, 0.895687, 0.896361, 0.897035, 0.897709, 0.898383, 0.899057, 0.899731,
            0.900405, 0.901079, 0.901753, 0.902427, 0.903101, 0.903775, 0.904449, 0.905123, 0.905797, 0.906471,
            0.907144, 0.907818, 0.908492, 0.909166, 0.909840, 0.910514, 0.911188, 0.911862, 0.912536, 0.913210,
            0.914006, 0.914945, 0.915885, 0.916824, 0.917764, 0.918703, 0.919642, 0.920582, 0.921521, 0.922461,
            0.923400, 0.924339, 0.925279, 0.926218, 0.927158, 0.928097, 0.929037, 0.929920, 0.930600, 0.931280,
            0.931960, 0.932641, 0.933321, 0.934001, 0.934681, 0.935361, 0.936041, 0.936721, 0.937402, 0.938082,
            0.938762, 0.939442, 0.940122, 0.940802, 0.941482, 0.942163, 0.942843, 0.943523, 0.944203, 0.944883,
            0.945563, 0.946243, 0.946924, 0.947604, 0.948284, 0.948964, 0.949644, 0.950324, 0.951004, 0.951685,
            0.952365, 0.952630, 0.953335, 0.954039, 0.954744, 0.955449, 0.956153, 0.956858, 0.957563, 0.958267,
            0.958972, 0.959677, 0.960381, 0.961086, 0.961791, 0.962495, 0.963200, 0.963905, 0.964610, 0.965314,
            0.966019, 0.966724, 0.967428, 0.968133, 0.968500, 0.969145, 0.969791, 0.970437, 0.971083, 0.971728,
            0.972374, 0.973020, 0.973665, 0.974311, 0.974957, 0.975602, 0.976248, 0.976310, 0.976939, 0.977568,
            0.978180, 0.978805, 0.979431, 0.980056, 0.980682, 0.981308, 0.981933, 0.982559, 0.983184, 0.983810,
            0.984435, 0.985061, 0.985686, 0.986312, 0.986938, 0.987563, 0.988189, 0.988814, 0.989440, 0.990065,
            0.990691, 0.991316, 0.991942, 0.992180, 0.992832, 0.993484, 0.994136, 0.994788, 0.995440, 0.996092,
            0.996744, 0.997395, 0.998047, 0.998699, 0.999351, 1.000000
        };

        bool isNeg = (targetY < 0.0);
        double absY = std::abs(targetY);

        if (absY <= 0.0) return 0.0;
        if (absY >= 1.0) return (isNeg ? -1.0 : 1.0);

        double pos = absY * 1000.0;
        int idx = static_cast<int>(pos);
        double frac = pos - idx;

        double x0 = x_lut[idx];
        double x1 = x_lut[idx + 1];
        return (isNeg ? -(x0 + (x1 - x0) * frac) : (x0 + (x1 - x0) * frac));
    }

    void publishInputs() {
        // Get steering value from lookup table
        const double steering_from_table = getRequiredX_Final(steering_.load());
        const double throttle = clamp(throttle_.load() * throttle_scale_, kTriggerMin, kTriggerMax);
        const double brake = clamp(brake_.load() * brake_scale_, kTriggerMin, kTriggerMax);

        const double rightStickY = clamp(brake - throttle, kAxisMin, kAxisMax);

        const int leftMapped = pad_.setLeftStickX(steering_from_table);
        const int rightMapped = pad_.setRightStickY(rightStickY);
        if (shouldReadPine()) {
            publishPineTelemetry();
        }
    }

    VirtualPad pad_{};
    PINE::PCSX2 ipc_{};
    PineAddresses addresses_{};
    std::chrono::steady_clock::time_point last_pine_read_ = std::chrono::steady_clock::now();
    const std::chrono::milliseconds pine_read_interval_{40};
    std::chrono::steady_clock::time_point last_speed_time_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point last_orient_time_ = std::chrono::steady_clock::now();
    float last_speed_ = 0.0f;
    float last_roll_ = 0.0f;
    float last_pitch_ = 0.0f;
    float last_yaw_ = 0.0f;
    bool has_last_speed_ = false;
    bool has_last_orient_ = false;
    std::atomic<double> steering_{0.0};
    std::atomic<double> throttle_{0.0};
    std::atomic<double> brake_{0.0};

    std::string steering_topic_;
    std::string throttle_topic_;
    std::string brake_topic_;
    std::string pos_x_topic_;
    std::string pos_y_topic_;
    std::string pos_z_topic_;
    std::string speed_topic_;
    std::string throttle_feedback_topic_;
    std::string brake_feedback_topic_;
    std::string accel_topic_;
    std::string roll_topic_;
    std::string pitch_topic_;
    std::string yaw_topic_;
    std::string steering_cmd_topic_;
    std::string steering_angle_topic_;
    std::string steering_angle_deg_topic_;
    std::string vehicle_state_topic_;
    double publish_rate_hz_ = 60.0;
    double throttle_scale_ = 1.0;
    double brake_scale_ = 1.0;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throttle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr brake_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pos_x_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pos_y_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pos_z_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_feedback_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr brake_feedback_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr accel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr roll_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_angle_deg_pub_;
    rclcpp::Publisher<vsac4gaming::msg::VehicleState>::SharedPtr vehicle_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VcuRosNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
