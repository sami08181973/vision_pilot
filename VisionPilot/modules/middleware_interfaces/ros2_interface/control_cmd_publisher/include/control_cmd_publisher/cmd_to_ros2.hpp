#ifndef VISIONPILOT_CMD_TO_ROS2_HPP
#define VISIONPILOT_CMD_TO_ROS2_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <cstdint>
#include <memory>
#include <thread>

namespace vp_middleware {

// ── POSIX shared-memory layout (/vp_ctrl_shm) ────────────────────────────────
// Same seqlock protocol as VehicleStateShmLayout.
// tyre_angle_rad  : signed tyre angle (rad, CCW-positive).
// acceleration_ms2: commanded acceleration (m/s²; negative = brake).
struct ControlCmdShmLayout {
    uint32_t epoch;
    float    tyre_angle_rad;
    float    acceleration_ms2;
    uint64_t timestamp_ns;
};
inline constexpr const char* VP_CTRL_SHM_NAME = "/vp_ctrl_shm";

// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class ControlCmdPublisher
 * @brief Accepts planner output (tyre_angle_rad, acceleration_ms2) and fans it
 *        out via two channels simultaneously:
 *
 *   1. ROS2 Float32 topics — consumed by the CARLA bridge or any ROS2 tool.
 *        /vehicle/steering_cmd  — tyre angle in rad
 *        /vehicle/throttle_cmd  — acceleration in m/s²
 *
 *   2. POSIX shmem /vp_ctrl_shm — zero-copy, no-ROS2-overhead read from any
 *        process (e.g., CAN writer, hardware bridge).
 *
 * Thread-safety: publish() can be called from any thread.
 */
class ControlCmdPublisher {
public:
    explicit ControlCmdPublisher(
        const std::string& steering_topic = "/vehicle/steering_cmd",
        const std::string& throttle_topic = "/vehicle/throttle_cmd",
        const std::string& node_name      = "control_cmd_publisher"
    );
    ~ControlCmdPublisher();

    // Publish tyre angle (rad) and acceleration (m/s²) to ROS2 + shmem.
    // Safe to call from the main loop thread.
    void publish(float tyre_angle_rad, float acceleration_ms2);

private:
    void write_to_shm(float tyre_angle_rad, float acceleration_ms2);
    void open_shm();

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
    std::thread spin_thread_;

    // POSIX shmem
    int   shm_fd_  = -1;
    void* shm_ptr_ = nullptr;
};

}  // namespace vp_middleware

#endif  // VISIONPILOT_CMD_TO_ROS2_HPP
