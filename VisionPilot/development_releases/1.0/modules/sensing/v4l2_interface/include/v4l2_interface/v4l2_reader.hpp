#ifndef VISIONPILOT_V4L2_READER_HPP
#define VISIONPILOT_V4L2_READER_HPP

#include <opencv2/opencv.hpp>
#include <cstdint>
#include <mutex>
#include <tuple>
#include <string>
#include <memory>
#include <iostream>

namespace v4l2_interface {

    class V4L2Reader {

        public:


        /**
        * @brief Constructor for V4L2Reader
        *
        * @param device_path Mounting path to V4L2 device (e.g., "/dev/video0")
        @ @param fps Desired FPS for outputing frames. Current default is 10.
        * 
        * Inits the V4L2 camera device with specified parameters and start capture/vis thread.
        * Also logs these details and subscribtion info.
        */    
        explicit V4L2Reader(
            const std::string& device_path,
            uint32_t fps = 10
        );


        /**
        * @brief Destructor for V4L2Reader
        *
        * Properly cleans up camera resources and closes video capture threads.
        */
        ~V4L2Reader();


        // FRAME HANDLINGS


        /**
        * @brief Get latest captured frame
        *
        * @return A tuple containing:
        *     - bool: true if frame is valid, false otherwise
        *     - cv::Mat: latest captured frame (empty if invalid)
        *
        * Thread-safe with the frame cloned and removed from buffer after retrieval.
        */
        std::tuple<bool, cv::Mat> get_latest_frame();


        /**
        * @brief Check if latest frame is currently available
        * 
        * @return true if a frame is available in the buffer, false otherwise
        */
        bool has_frames() const;


        /**
        * @brief Reset frame buffer (clear latest frame slot)
        * 
        * Kinda useful for resetting state or handling error conditions
        */
        void clear_frame_buffer();


        /**
        * @brief Check if V4L2 camera stream is active
        * 
        * @return true if stream has started capturing frames, false otherwise
        */
        bool is_stream_active() const;


        /**
        * @brief Check if V4L2 device is properly opened
        * 
        * @return true if VideoCapture is successfully opened, false otherwise
        */
        bool is_device_open() const;


        

    }

}

#endif //VISIONPILOT_V4L2_READER_HPP
