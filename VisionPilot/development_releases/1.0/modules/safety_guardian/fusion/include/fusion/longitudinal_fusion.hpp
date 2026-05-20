#pragma once

#include <models/auto_drive.hpp>
#include <models/auto_speed.hpp>
#include <opencv2/core.hpp>

#include <memory>
#include <random>
#include <string>
#include <vector>

// ObjectFinder lives in fusion/tracking/ — forward-declared here to keep this
// header free of OpenCV/tracking internals for downstream consumers.
namespace visionpilot::tracking { class ObjectFinder; }

namespace visionpilot::fusion {

// ─── Output ────────────────────────────────────────────────────────────────────
// Self-contained — no tracking types exposed publicly.
struct CIPOFusionEstimate {
    bool  valid              = false;

    // Particle-filter (fused) posterior
    float distance_m         = 0.f;
    float velocity_ms        = 0.f;    // negative = approaching
    float distance_stddev_m  = 0.f;
    float velocity_stddev_ms = 0.f;

    // Kalman-tracker snapshot (for display / cut-in alerting)
    bool  tracker_found      = false;
    int   tracker_id         = -1;
    float tracker_dist_m     = 0.f;
    float tracker_vel_ms     = 0.f;
    bool  cut_in_detected    = false;
};

// ─── LongitudinalFusion ────────────────────────────────────────────────────────
//
// Single entry point for CIPO longitudinal estimation.  Internally handles:
//   1. ObjectFinder Kalman tracker (if homography_path configured)
//         AutoSpeed detections + homography → tracked CIPO distance
//   2. Particle filter fusing AutoDrive + tracker into final distance/velocity
//
// Usage:
//   LongitudinalFusion::Config cfg;
//   cfg.homography_path = "/path/to/homography_zod.yaml";
//   LongitudinalFusion fusion(cfg);
//
//   // Per frame (after inference barrier):
//   CIPOFusionEstimate est = fusion.update(drive_out, speed_out, original_frame);
//
class LongitudinalFusion {
public:
    struct Config {
        int   n_particles          = 500;
        float d_max_m              = 150.f;
        float dt_s                 = 0.10f;   // nominal dt; overridden per-call
        float process_noise_dist_m = 0.50f;
        float process_noise_vel_ms = 0.20f;
        float autodrive_noise_m    = 15.f;
        float tracker_noise_m      = 3.f;
        std::string homography_path = "";     // enable ObjectFinder when non-empty
        // Per-frame log: AutoDrive_d | Tracker_d Tracker_v | Fused_d Fused_v ±std
        bool debug = false;
    };

    LongitudinalFusion();
    explicit LongitudinalFusion(Config cfg);
    ~LongitudinalFusion();  // defined in .cpp where ObjectFinder is complete

    // Move-only (unique_ptr member).
    // Definitions live in longitudinal_fusion.cpp where ObjectFinder is complete.
    LongitudinalFusion(LongitudinalFusion&&) noexcept;
    LongitudinalFusion& operator=(LongitudinalFusion&&) noexcept;

    // Main per-frame call.
    // autodrive         : output from AutoDrive model
    // autospeed         : output from AutoSpeed model (bboxes already in 1024×512 space)
    // preprocessed_frame: the center-cropped+resized 1024×512 frame fed to the models
    //                     (same coordinate space as the homography calibration)
    // dt_s              : elapsed seconds since last call; 0 → use cfg.dt_s
    CIPOFusionEstimate update(
        const models::AutoDriveOutput& autodrive,
        const models::AutoSpeedOutput& autospeed,
        const cv::Mat& preprocessed_frame,
        float dt_s = 0.f);

    void reset();
    const Config& config() const { return cfg_; }

private:
    // log_w follows the MRPT convention: accumulated in log-space between resamples,
    // reset to 0 after each systematic resample.  Never stored as linear to avoid
    // exp() underflow (which caused the catastrophic collapse to 0 m).
    struct Particle { float distance_m, velocity_ms, log_w; };

    // Internal measurement bundle
    struct Meas { float distance_m = 0.f; float stddev_m = 15.f; bool valid = false; };

    void  init_from(float dist_m, float stddev_m);
    void  predict(float dt_s);
    void  weight_update(const Meas& ad, const Meas& tr);
    // Convert log-weights to normalised linear weights using the log-sum-exp trick.
    // Never underflows — always returns a valid probability distribution.
    std::vector<float> linear_weights() const;
    float effective_n() const;
    void  resample();
    static float gaussian_loglik(float z, float mean, float sigma);

    Config cfg_;
    std::vector<Particle> particles_;
    bool  initialised_ = false;
    std::mt19937 rng_;

    // ObjectFinder — null until first update() call when homography_path is set.
    std::unique_ptr<tracking::ObjectFinder> tracker_;
};

}  // namespace visionpilot::fusion
