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
#include <algorithm>

namespace {
constexpr double kAxisMin = -1.0;
constexpr double kAxisMax = 1.0;
constexpr double kTriggerMin = 0.0;
constexpr double kTriggerMax = 1.0;
constexpr float kRadToDeg = 57.2957795f;

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
        msg.data = telemetry.steering_angle_deg;
        steering_angle_deg_pub_->publish(msg);
    }

/**
 * Calculates Required X for Target Y in range [-1.0, 1.0].
 * Updated LUT using normalized values from.
 */
double getRequiredX_HighRes(double targetY) {
    // 1001 entries where index = |targetY| * 1000
    // Maps Target Y (index) to Required X (value)
    static const double x_lut[1001] = {
        0.000000, 0.039173, 0.078346, 0.117518, 0.156691, 0.195864, 0.235036, 0.274209, 0.307137, 0.308332,
        0.309527, 0.310722, 0.311918, 0.313113, 0.314308, 0.315503, 0.316698, 0.317894, 0.319089, 0.320284,
        0.321479, 0.322675, 0.323870, 0.325065, 0.326260, 0.327455, 0.328651, 0.329846, 0.331041, 0.332236,
        0.333431, 0.334627, 0.335822, 0.337017, 0.338212, 0.339408, 0.340603, 0.341798, 0.342993, 0.344188,
        0.345384, 0.346579, 0.347774, 0.348969, 0.350165, 0.351360, 0.352555, 0.353750, 0.354945, 0.356141,
        0.357336, 0.358531, 0.359726, 0.360922, 0.362117, 0.363312, 0.364507, 0.365702, 0.366898, 0.368093,
        0.369288, 0.370483, 0.371679, 0.372338, 0.373021, 0.373703, 0.374386, 0.375069, 0.375751, 0.376434,
        0.377117, 0.377799, 0.378482, 0.379165, 0.379848, 0.380530, 0.381213, 0.381896, 0.382578, 0.383261,
        0.383944, 0.384626, 0.385309, 0.385992, 0.386675, 0.387357, 0.388102, 0.388804, 0.389505, 0.390206,
        0.390907, 0.391609, 0.392310, 0.393011, 0.393712, 0.394413, 0.395115, 0.395816, 0.396517, 0.397218,
        0.397919, 0.398621, 0.399322, 0.400023, 0.400724, 0.401425, 0.402127, 0.402828, 0.403529, 0.404230,
        0.404931, 0.405633, 0.406334, 0.407035, 0.407736, 0.408438, 0.409139, 0.409840, 0.410541, 0.411242,
        0.411944, 0.412645, 0.413346, 0.414047, 0.414748, 0.415450, 0.416151, 0.416852, 0.417553, 0.418254,
        0.418956, 0.419657, 0.420358, 0.421059, 0.421760, 0.422462, 0.423163, 0.423864, 0.424565, 0.425267,
        0.425968, 0.426669, 0.427370, 0.428071, 0.428773, 0.429474, 0.430175, 0.430876, 0.431577, 0.432279,
        0.433054, 0.433816, 0.434579, 0.435342, 0.436104, 0.436867, 0.437630, 0.438392, 0.439155, 0.439918,
        0.440681, 0.441443, 0.442206, 0.442969, 0.443731, 0.444494, 0.445257, 0.446019, 0.446782, 0.447545,
        0.448308, 0.449070, 0.449833, 0.450596, 0.451358, 0.452121, 0.452884, 0.453646, 0.454409, 0.455172,
        0.455935, 0.456697, 0.457460, 0.458223, 0.458985, 0.459748, 0.460511, 0.461273, 0.462036, 0.462799,
        0.463562, 0.464324, 0.465087, 0.465850, 0.466612, 0.467375, 0.468138, 0.468900, 0.469663, 0.470426,
        0.471189, 0.471951, 0.472714, 0.473477, 0.474239, 0.475002, 0.475765, 0.476527, 0.477290, 0.478053,
        0.478815, 0.479578, 0.480341, 0.481104, 0.481866, 0.482613, 0.483344, 0.484074, 0.484805, 0.485536,
        0.486266, 0.486997, 0.487728, 0.488458, 0.489189, 0.489919, 0.490650, 0.491381, 0.492111, 0.492842,
        0.493573, 0.494303, 0.495034, 0.495764, 0.496495, 0.497226, 0.497956, 0.498687, 0.499418, 0.500148,
        0.500879, 0.501609, 0.502340, 0.503071, 0.503801, 0.504532, 0.505263, 0.505993, 0.506724, 0.507455,
        0.508185, 0.508916, 0.509646, 0.510377, 0.511108, 0.511838, 0.512569, 0.513299, 0.514030, 0.514761,
        0.515491, 0.516222, 0.516953, 0.517683, 0.518414, 0.519145, 0.519875, 0.520606, 0.521319, 0.521509,
        0.521698, 0.521888, 0.522077, 0.522267, 0.522456, 0.522646, 0.522835, 0.523025, 0.523214, 0.523404,
        0.523594, 0.523783, 0.523973, 0.524162, 0.524352, 0.524541, 0.524731, 0.524920, 0.525110, 0.525299,
        0.525489, 0.525678, 0.525868, 0.526057, 0.526247, 0.526437, 0.526626, 0.526816, 0.527005, 0.527195,
        0.527384, 0.527574, 0.527763, 0.527953, 0.528142, 0.528332, 0.528521, 0.528711, 0.528900, 0.529090,
        0.529279, 0.529469, 0.529658, 0.529848, 0.530038, 0.530227, 0.530417, 0.530606, 0.530796, 0.530985,
        0.531175, 0.531364, 0.531554, 0.531743, 0.531933, 0.532122, 0.532312, 0.532502, 0.532691, 0.532881,
        0.533070, 0.533260, 0.533449, 0.533639, 0.533828, 0.534018, 0.534241, 0.534791, 0.535341, 0.535891,
        0.536441, 0.536991, 0.537541, 0.538091, 0.538641, 0.539191, 0.539741, 0.540291, 0.540841, 0.541391,
        0.541941, 0.542491, 0.543041, 0.543591, 0.544141, 0.544691, 0.545241, 0.545791, 0.546341, 0.546891,
        0.547441, 0.547991, 0.548541, 0.549091, 0.549641, 0.550191, 0.550741, 0.551291, 0.551841, 0.552391,
        0.552941, 0.553491, 0.554041, 0.554591, 0.555141, 0.555691, 0.556241, 0.556791, 0.557341, 0.557891,
        0.558441, 0.558991, 0.559541, 0.560091, 0.560641, 0.561191, 0.561741, 0.562291, 0.562841, 0.563391,
        0.563941, 0.564491, 0.565041, 0.565591, 0.566141, 0.566691, 0.567241, 0.567791, 0.568341, 0.568891,
        0.569441, 0.569991, 0.570541, 0.571091, 0.571641, 0.572191, 0.572741, 0.573291, 0.573841, 0.574391,
        0.574941, 0.575491, 0.576041, 0.576591, 0.577141, 0.577691, 0.578241, 0.578791, 0.579341, 0.579891,
        0.580441, 0.580991, 0.581541, 0.582091, 0.582641, 0.583191, 0.583741, 0.584291, 0.584841, 0.585391,
        0.585941, 0.586491, 0.587041, 0.587591, 0.588141, 0.588691, 0.589241, 0.589791, 0.590325, 0.590878,
        0.591431, 0.591984, 0.592537, 0.593090, 0.593643, 0.594196, 0.594749, 0.595302, 0.595855, 0.596408,
        0.596961, 0.597514, 0.598067, 0.598620, 0.599173, 0.599726, 0.600279, 0.600832, 0.601385, 0.601938,
        0.602491, 0.603044, 0.603597, 0.604150, 0.604703, 0.605256, 0.605809, 0.606362, 0.606915, 0.607468,
        0.608021, 0.608574, 0.609127, 0.609680, 0.610233, 0.610786, 0.611339, 0.611892, 0.612445, 0.612998,
        0.613551, 0.614104, 0.614657, 0.615210, 0.615763, 0.616316, 0.616353, 0.617066, 0.617780, 0.618494,
        0.619208, 0.619922, 0.620635, 0.621349, 0.622063, 0.622777, 0.623490, 0.624204, 0.624918, 0.625632,
        0.626346, 0.627059, 0.627773, 0.628487, 0.629201, 0.629915, 0.630628, 0.631342, 0.632056, 0.632770,
        0.633484, 0.634197, 0.634911, 0.635625, 0.636339, 0.637052, 0.637766, 0.638480, 0.639194, 0.639908,
        0.640621, 0.641335, 0.642049, 0.642763, 0.643477, 0.644190, 0.644904, 0.645618, 0.646332, 0.647046,
        0.647759, 0.648473, 0.649187, 0.649901, 0.650615, 0.650837, 0.651761, 0.652684, 0.653607, 0.654530,
        0.655454, 0.656377, 0.657300, 0.658223, 0.659147, 0.660070, 0.660993, 0.661916, 0.662840, 0.663763,
        0.664686, 0.665609, 0.666533, 0.667456, 0.668379, 0.669302, 0.670226, 0.671149, 0.672072, 0.672995,
        0.673919, 0.674842, 0.675765, 0.676688, 0.677612, 0.678535, 0.679458, 0.680381, 0.681305, 0.682228,
        0.683151, 0.684074, 0.684998, 0.685921, 0.686844, 0.687767, 0.688691, 0.689614, 0.690537, 0.691460,
        0.692384, 0.692440, 0.693245, 0.694051, 0.694857, 0.695662, 0.696468, 0.697274, 0.698080, 0.698885,
        0.699691, 0.700497, 0.701303, 0.702108, 0.702914, 0.703720, 0.704526, 0.705331, 0.706137, 0.706943,
        0.707749, 0.708554, 0.709360, 0.710166, 0.710971, 0.711777, 0.712583, 0.713401, 0.714243, 0.715085,
        0.715927, 0.716769, 0.717611, 0.718453, 0.719295, 0.720137, 0.720979, 0.721821, 0.722663, 0.723505,
        0.724347, 0.725189, 0.726031, 0.726873, 0.727715, 0.728557, 0.729399, 0.730241, 0.731083, 0.731925,
        0.732767, 0.733609, 0.734451, 0.735293, 0.736135, 0.736977, 0.737819, 0.738661, 0.739503, 0.740345,
        0.741187, 0.742029, 0.742871, 0.743713, 0.744555, 0.745397, 0.746239, 0.747081, 0.747923, 0.748765,
        0.749607, 0.750449, 0.751291, 0.752133, 0.752975, 0.753817, 0.754659, 0.755501, 0.756343, 0.757185,
        0.758027, 0.758869, 0.759711, 0.760553, 0.761395, 0.762237, 0.763079, 0.763921, 0.764763, 0.765605,
        0.766447, 0.767289, 0.768131, 0.768973, 0.769815, 0.770657, 0.771499, 0.772341, 0.773183, 0.774025,
        0.774867, 0.775709, 0.776551, 0.777393, 0.778235, 0.779077, 0.779919, 0.780761, 0.781603, 0.782210,
        0.782944, 0.783678, 0.784411, 0.785145, 0.785878, 0.786612, 0.787345, 0.788079, 0.788812, 0.789546,
        0.790279, 0.791013, 0.791746, 0.792480, 0.793213, 0.793947, 0.794680, 0.795414, 0.796147, 0.796881,
        0.797614, 0.798348, 0.799081, 0.799815, 0.800548, 0.801282, 0.802015, 0.802749, 0.803482, 0.804216,
        0.804949, 0.805683, 0.806416, 0.807150, 0.807883, 0.808617, 0.809350, 0.810084, 0.810817, 0.811551,
        0.812284, 0.813018, 0.813751, 0.814485, 0.815218, 0.815952, 0.816685, 0.817419, 0.818152, 0.818886,
        0.819619, 0.820353, 0.821086, 0.821820, 0.822553, 0.823287, 0.824020, 0.824754, 0.825487, 0.826221,
        0.826954, 0.827688, 0.828421, 0.830230, 0.830964, 0.831697, 0.832431, 0.833164, 0.833898, 0.834631,
        0.835365, 0.836098, 0.836832, 0.837565, 0.838299, 0.839032, 0.839766, 0.840499, 0.841233, 0.841966,
        0.842699, 0.843433, 0.844166, 0.844900, 0.845633, 0.846367, 0.847100, 0.847834, 0.848567, 0.849301,
        0.850034, 0.850768, 0.851501, 0.852235, 0.852968, 0.853702, 0.853965, 0.854652, 0.855339, 0.856027,
        0.856714, 0.857401, 0.858088, 0.858776, 0.859463, 0.860150, 0.860838, 0.861525, 0.862212, 0.862899,
        0.863587, 0.864274, 0.864961, 0.865649, 0.866336, 0.867023, 0.867710, 0.868398, 0.869085, 0.869772,
        0.870459, 0.871147, 0.871834, 0.872521, 0.873209, 0.873896, 0.874583, 0.875270, 0.875958, 0.876645,
        0.877332, 0.878019, 0.878707, 0.879394, 0.880081, 0.880769, 0.881456, 0.882143, 0.882830, 0.883518,
        0.884205, 0.884892, 0.885579, 0.886267, 0.886954, 0.887641, 0.888329, 0.889016, 0.889703, 0.890390,
        0.891078, 0.891765, 0.892452, 0.893139, 0.893827, 0.893988, 0.894762, 0.895536, 0.896309, 0.897083,
        0.897857, 0.898630, 0.899404, 0.900178, 0.900952, 0.901725, 0.902499, 0.903273, 0.904046, 0.904820,
        0.905594, 0.906368, 0.907141, 0.907915, 0.908689, 0.909462, 0.910236, 0.911010, 0.911783, 0.912557,
        0.913331, 0.914105, 0.914878, 0.915652, 0.916426, 0.917199, 0.917973, 0.918747, 0.919521, 0.920294,
        0.921068, 0.921842, 0.922615, 0.923389, 0.924163, 0.924937, 0.925710, 0.926484, 0.927258, 0.928031,
        0.928805, 0.929579, 0.929928, 0.930609, 0.931291, 0.931973, 0.932655, 0.933336, 0.934018, 0.934700,
        0.935381, 0.936063, 0.936745, 0.937427, 0.938108, 0.938790, 0.939472, 0.940153, 0.940835, 0.941517,
        0.942199, 0.942880, 0.943562, 0.944244, 0.944926, 0.945607, 0.946289, 0.946971, 0.947652, 0.948334,
        0.949016, 0.949698, 0.950379, 0.951061, 0.951743, 0.952425, 0.953106, 0.953788, 0.954470, 0.955152,
        0.955833, 0.956515, 0.957197, 0.957819, 0.958514, 0.959210, 0.959905, 0.960601, 0.961296, 0.961992,
        0.962687, 0.963383, 0.964078, 0.964774, 0.965469, 0.966165, 0.966861, 0.967556, 0.968252, 0.968947,
        0.969643, 0.970338, 0.971034, 0.971729, 0.972425, 0.973120, 0.973816, 0.974512, 0.975207, 0.975903,
        0.976598, 0.977294, 0.977989, 0.978183, 0.978864, 0.979545, 0.980226, 0.980907, 0.981588, 0.982269,
        0.982950, 0.983632, 0.984313, 0.984994, 0.985675, 0.986356, 0.987037, 0.987718, 0.988399, 0.989080,
        0.989761, 0.990442, 0.991123, 0.991804, 0.992485, 0.993166, 0.993847, 0.994528, 0.995209, 0.995890,
        0.996571, 0.997252, 0.997933, 0.998614, 0.999295, 1.000000
    };

    // 1. Polarity check
    bool isNeg = (targetY < 0.0);
    double absY = std::abs(targetY);

    // 2. Boundary Clamping
    if (absY <= 0.0) return 0.0;
    if (absY >= 1.0) return (isNeg ? -1.0 : 1.0);

    // 3. Precise Indexing (1000 intervals for 1001 points)
    double pos = absY * 1000.0;
    int idx = static_cast<int>(pos);
    double frac = pos - idx;

    // 4. Linear Interpolation
    double x0 = x_lut[idx];
    double x1 = x_lut[idx + 1];
    double reqX = x0 + (x1 - x0) * frac;

    return (isNeg ? -reqX : reqX);
}
    void publishInputs() {
        const double steering = steering_.load();
        const double throttle = clamp(throttle_.load() * throttle_scale_, kTriggerMin, kTriggerMax);
        const double brake = clamp(brake_.load() * brake_scale_, kTriggerMin, kTriggerMax);

        const double rightStickY = clamp(brake - throttle, kAxisMin, kAxisMax);
        
        // Get steering value from lookup table
        const double steering_from_table = getRequiredX_HighRes(steering);
        // Todo: remove debug prints
        printf("Target Steering: %f\n", steering);
        printf("Steering from table: %f\n", steering_from_table);

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
    std::string steering_angle_deg_topic_;
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
