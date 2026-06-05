// VisionPilot — preprocess → inference → fusion → display
#include <config/vision_pilot_config.hpp>
#include <debug/debug_draw.hpp>
#include <engine/onnx_engine.hpp>
#include <image_preprocessing/image_preprocessor.hpp>
#include <logging/logger.hpp>
#include <models/inference.hpp>
#include <visualization/visualization.hpp>

#include <camera_interface/camera_interface.hpp>
#include <camera_interface/v4l2_camera_interface.hpp>
#ifdef ENABLE_WEBRTC
#include <visualization/visualization_to_webrtc.hpp>
#endif
#ifdef ENABLE_ROS2_INTERFACE
#include <camera_subscriber/ros2_to_opencv.hpp>
#endif

#include <chrono>
#include <memory>
#include <opencv2/videoio.hpp>
#include <thread>

namespace ve = visionpilot::engine;
namespace vm = visionpilot::models;
namespace vd = visionpilot::debug;

int main(int argc, char** argv)
{
    // ── 1. Config ─────────────────────────────────────────────────────────────
    const std::string cfg_path = resolve_vision_pilot_config_path(argc, argv);
    if (cfg_path.empty()) {
        VP_ERROR("No config — cp config/vision_pilot.conf.example config/vision_pilot.conf");
        return 1;
    }

    VisionPilotConfig cfg;
    try { cfg = load_vision_pilot_config(cfg_path); }
    catch (const std::exception& e) { VP_ERROR("Config: %s", e.what()); return 1; }

    // ── 2. Pipeline (preprocess + ONNX + inference/fusion) ────────────────────
    ImagePreprocessor preprocessor;
    ve::OnnxEngine engine(cfg.engine_cfg);
    vm::InferencePipeline pipeline(engine, {
        cfg.autodrive_model, cfg.autosteer_model, cfg.autospeed_model,
        cfg.homography_path, cfg.fusion_debug,
    });

    vd::init_wheel_assets(cfg.wheel_dir);
    vd::init_homography(cfg.homography_path);

    // ── 3. Display output ─────────────────────────────────────────────────────
    bool show_window = true;
#ifdef ENABLE_WEBRTC
    std::unique_ptr<visualization::WebRTCStreamer> webrtc;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--webrtc") show_window = false;
        if (std::string(argv[i]) == "--webrtc-port" && i + 1 < argc) {
            webrtc = std::make_unique<visualization::WebRTCStreamer>();
            if (!webrtc->init(static_cast<uint16_t>(std::stoi(argv[++i])))) return 1;
        }
    }
#endif

    const cv::Size net_size(vm::AutoDrive::NET_W, vm::AutoDrive::NET_H);
    cv::Mat frame, warped, resized;

    // ── 4. Main loop — video file ─────────────────────────────────────────────
    if (cfg.source.mode == SourceMode::Video) {
        cv::VideoCapture cap(cfg.source.video_path);
        if (!cap.isOpened()) {
            VP_ERROR("Cannot open video: %s", cfg.source.video_path.c_str());
            return 1;
        }

        const double fps = cap.get(cv::CAP_PROP_FPS);
        const auto period = (cfg.source.video_realtime && fps > 1.0)
                                ? std::chrono::duration<double>(1.0 / fps)
                                : std::chrono::duration<double>(0);

        for (;;) {
            const auto t0 = std::chrono::steady_clock::now();

            // acquire
            if (!cap.read(frame) || frame.empty()) {
                if (!cfg.source.video_loop) break;
                cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                pipeline.reset();
                continue;
            }

            // preprocess
            preprocessor.preprocess(frame, warped, resized, net_size);

            // inference + fusion
            if (const auto r = pipeline.process(warped)) {
                if (r->frame_id % 30 == 0) pipeline.latency().print();
                vd::annotate_frame(warped, {
                    r->frame_id, r->wall_ms, r->pre_ms, r->ad_ms, r->as_ms, r->asp_ms,
                    "video",
                    r->auto_drive, r->auto_steer, r->auto_speed,
                    r->cipo, r->lateral,
                    {}, cfg.wheel_dir, cfg.homography_path,
                });
            }

            // display
            if (show_window) visualization::render_frame(warped, "VisionPilot", {});
#ifdef ENABLE_WEBRTC
            if (webrtc) webrtc->push_frame(warped);
#endif

            if (period.count() > 0) {
                const auto rem = period - (std::chrono::steady_clock::now() - t0);
                if (rem.count() > 0) std::this_thread::sleep_for(rem);
            }
        }
        visualization::close_windows();
        return 0;
    }

    // ── 4. Main loop — live camera (ROS2 / V4L2) ────────────────────────────
    camera_interface::CameraInterface* cam = nullptr;
    camera_interface::V4L2CameraInterface v4l2_cam(cfg.source.v4l2_device,
                                                   static_cast<uint32_t>(cfg.source.v4l2_fps));
#ifdef ENABLE_ROS2_INTERFACE
    camera_interface::ROS2ImageSubscriber ros2_cam(cfg.source.ros2_topic);
#endif

    const char* src_label = "";
    if (cfg.source.mode == SourceMode::V4l2) {
        cam = &v4l2_cam;
        src_label = cfg.source.v4l2_device.c_str();
    } else if (cfg.source.mode == SourceMode::Ros2) {
#ifdef ENABLE_ROS2_INTERFACE
        cam = &ros2_cam;
        src_label = cfg.source.ros2_topic.c_str();
#else
        VP_ERROR("ROS2 requested but ENABLE_ROS2_INTERFACE=OFF");
        return 1;
#endif
    }

    for (;;) {
        // acquire
        auto [ok, frame] = cam->get_latest_frame();
        if (!ok || frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // preprocess
        preprocessor.preprocess(frame, warped, resized, net_size);

        // inference + fusion
        if (const auto r = pipeline.process(warped)) {
            if (r->frame_id % 30 == 0) pipeline.latency().print();
            vd::annotate_frame(warped, {
                r->frame_id, r->wall_ms, r->pre_ms, r->ad_ms, r->as_ms, r->asp_ms,
                src_label,
                r->auto_drive, r->auto_steer, r->auto_speed,
                r->cipo, r->lateral,
                {}, cfg.wheel_dir, cfg.homography_path,
            });
        }

        // display
        if (show_window) visualization::render_frame(warped, "VisionPilot", {});
#ifdef ENABLE_WEBRTC
        if (webrtc) webrtc->push_frame(warped);
#endif
    }
}
