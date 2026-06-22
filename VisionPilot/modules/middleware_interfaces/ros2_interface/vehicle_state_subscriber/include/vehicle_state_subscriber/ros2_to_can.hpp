#ifndef VISIONPILOT_ROS2_TO_CAN_HPP
#define VISIONPILOT_ROS2_TO_CAN_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

namespace vp_middleware {

// ── In-process state (returned to callers) ───────────────────────────────────
struct VehicleState {
    float speed_ms       = 0.0f;  // ego speed  (m/s)
    float steering_rad   = 0.0f;  // wheel angle (rad, CCW-positive)
};

// ── POSIX shared-memory layout (/vp_state_shm) ───────────────────────────────
// Seqlock protocol: writer bumps epoch to odd before write, even after.
// Reader: read epoch → read data → read epoch; retry if odd or changed.
struct VehicleStateShmLayout {
    uint32_t epoch;          // seqlock counter
    float    speed_ms;
    float    steering_rad;
    uint64_t timestamp_ns;
};
inline constexpr const char* VP_STATE_SHM_NAME = "/vp_state_shm";

// ─────────────────────────────────────────────────────────────────────────────

/**
 * @class VehicleStateSubscriber
 * @brief ROS2 node that subscribes to /vehicle/speed and /vehicle/steering_angle,
 *        exposes the latest state in-process via get_state(), and writes the same
 *        data to the POSIX shared-memory segment /vp_state_shm so any other
 *        process can read it without a ROS2 dependency.
 *
 * Topics (default):
 *   /vehicle/speed           std_msgs/Float32  — ego speed in m/s
 *   /vehicle/steering_angle  std_msgs/Float32  — wheel angle in rad
 *
 * The node spins in a background thread (same pattern as camera_subscriber).
 * Thread-safety: get_state() / is_valid() are guarded by a mutex.
 */
class VehicleStateSubscriber {
public:
    explicit VehicleStateSubscriber(
        const std::string& speed_topic    = "/vehicle/speed",
        const std::string& steering_topic = "/vehicle/steering_angle",
        const std::string& node_name      = "vehicle_state_subscriber"
    );
    ~VehicleStateSubscriber();

    // Returns the latest received state (thread-safe).
    VehicleState get_state() const;

    // True once at least one message has been received on both topics.
    bool is_valid() const;

private:
    void speed_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void steering_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void write_to_shm(const VehicleState& s);
    void open_shm();

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub_;
    std::thread spin_thread_;

    mutable std::mutex state_mutex_;
    VehicleState state_;
    bool got_speed_    = false;
    bool got_steering_ = false;

    // POSIX shmem
    int   shm_fd_  = -1;
    void* shm_ptr_ = nullptr;
};

}  // namespace vp_middleware

#endif  // VISIONPILOT_ROS2_TO_CAN_HPP
