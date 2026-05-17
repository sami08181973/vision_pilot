#pragma once

#include <engine/onnx_engine.hpp>

#include <string>

namespace vpe = visionpilot::engine;

enum class SourceMode {
    Ros2  = 0,
    V4l2  = 1,
    Video = 2,
};

struct SourceConfig {
  SourceMode mode = SourceMode::Video;

  // video (mode 2)
  std::string video_path;
  bool        video_realtime = true;  // pace to file FPS
  bool        video_loop     = false;

  // ros2 (mode 0)
  std::string ros2_topic = "/camera/image";

  // v4l2 (mode 1)
  std::string v4l2_device = "/dev/video0";
  int         v4l2_fps    = 10;
};

struct PipelineConfig {
  // After models load, read two frames and run one synced inference; print summary.
  bool initial_inference_check = true;
};

// Application settings loaded from vision_pilot.conf (key = value format).
struct VisionPilotConfig {
    std::string          autodrive_model;
    std::string          autosteer_model;
    std::string          autospeed_model;
    vpe::EngineConfig    engine_cfg;
    SourceConfig         source;
    PipelineConfig       pipeline;
};

// Load config from a .conf file. Expands leading ~ to $HOME.
// Throws std::runtime_error if the file is missing or required keys are absent.
VisionPilotConfig load_vision_pilot_config(const std::string& config_path);

// Resolve config path: --config <path>, then $VISIONPILOT_CONFIG,
// then ./config/vision_pilot.conf, then ./vision_pilot.conf.
// Returns empty string if nothing was found.
std::string resolve_vision_pilot_config_path(int argc, char** argv);

// Parse source.mode string from config ("video", "ros2", "v4l2", or 0/1/2).
SourceMode parse_source_mode(const std::string& value);
