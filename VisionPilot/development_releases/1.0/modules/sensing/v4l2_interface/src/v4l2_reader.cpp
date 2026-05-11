#include <v4l2_interface/v4l2_reader.hpp>

#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>


namespace v4l2_interface {

    V4L2Reader::V4L2Reader(
        const std::string& device_path,
        uint32_t fps
    ) : device_path(device_path),
        target_fps(fps),
        is_stream_started(false),
        has_latest_frame(false)
    {
        
        log_info("Initializing V4L2 Reader");
        log_info("  Device Path: " + device_path);
        log_info("  Target FPS: " + std::to_string(fps));

        stats.device_path = device_path;
        stats.current_fps = fps;

        // Init display rate throttling
        display_frame_period_ms = (fps > 0) ? (1000 / fps) : 33;
        last_display_time = std::chrono::steady_clock::now();

        // Parse device number from path (for example, "/dev/video0" => 0)
        int device_number = 0;
        try {
            size_t last_slash = device_path.find_last_of('/');
            if (last_slash != std::string::npos) {
                std::string dev_num_str = device_path.substr(last_slash + 1);
                if (dev_num_str.find("video") != std::string::npos) {
                    device_number = std::stoi(dev_num_str.substr(5));
                }
            }
        } catch (const std::exception& e) {
            log_warning("Could not parse device number from path: " + device_path);
        }

        

}