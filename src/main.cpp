#include "pine.h"
#include "virtual_pad.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

namespace {
constexpr double kAxisMin = -1.0;
constexpr double kAxisMax = 1.0;
constexpr double kTriggerMin = 0.0;
constexpr double kTriggerMax = 1.0;

constexpr double clamp(double value, double minValue, double maxValue) {
    return value < minValue ? minValue : (value > maxValue ? maxValue : value);
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
        if (has_last_speed_) {
            const std::chrono::duration<float> dt = now - last_speed_time_;
            if (dt.count() > 0.0f) {
                accel = (telemetry.speed - last_speed_) / dt.count();
            }
        }
        last_speed_ = telemetry.speed;
        last_speed_time_ = now;
        has_last_speed_ = true;

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
        msg.data = telemetry.yaw;
        yaw_pub_->publish(msg);
        msg.data = telemetry.steering_command;
        steering_cmd_pub_->publish(msg);
        msg.data = telemetry.steering_angle_rad;
        steering_angle_pub_->publish(msg);
    }

    void publishInputs() {
        const double steering = steering_.load();
        const double throttle = clamp(throttle_.load() * throttle_scale_, kTriggerMin, kTriggerMax);
        const double brake = clamp(brake_.load() * brake_scale_, kTriggerMin, kTriggerMax);

        const double rightStickY = clamp(brake - throttle, kAxisMin, kAxisMax);
        const int leftMapped = pad_.setLeftStickX(steering);
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
    float last_speed_ = 0.0f;
    bool has_last_speed_ = false;
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
