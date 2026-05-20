#pragma once

#include <cstdint>
#include <random>
#include <stdexcept>
#include <vector>

namespace visionpilot::fusion {

// ─── Input type ────────────────────────────────────────────────────────────────
// Caller converts model outputs to this before each update() call.
// Set valid=false for any source that should be skipped this frame.
struct DistanceMeasurement {
    float distance_m = 0.f;   // estimated distance in metres  [0, D_MAX]
    float stddev_m   = 15.f;  // 1-sigma Gaussian noise characterisation
    bool  valid      = false;
};

// ─── Output type ───────────────────────────────────────────────────────────────
// velocity_ms sign convention (matches ObjectFinder):
//   negative → object approaching (distance decreasing)
//   positive → object receding   (distance increasing)
struct CIPOFusionEstimate {
    bool  valid              = false;
    float distance_m         = 0.f;  // particle-weighted mean distance (m)
    float velocity_ms        = 0.f;  // particle-weighted mean velocity (m/s)
    float distance_stddev_m  = 0.f;  // spread of the distance distribution
    float velocity_stddev_ms = 0.f;  // spread of the velocity distribution
};

// ─── Particle filter ───────────────────────────────────────────────────────────
//
// Fuses two independent distance sources for the CIPO (Closest In-Path Object):
//
//   Source A — AutoDrive model:
//     dist_normalized ∈ [0,1]  →  distance_m = D_MAX * (1 – dist_normalized)
//     Always applied (noisier, wide field of view, frame-to-frame stable).
//
//   Source B — Kalman tracker (ObjectFinder, ported in future iterations):
//     Precise per-track distance estimate; passed as optional DistanceMeasurement.
//     Tighter noise budget; significantly anchors the distribution when present.
//
// State vector per particle:  [distance_m, velocity_ms]
// Process model:              constant velocity + independent Gaussian noise
// Measurement model:          Gaussian likelihood; log-space for stability
// Resampling:                 systematic low-variance when Neff < N/2
//
class LongitudinalFusion {
public:
    struct Config {
        int   n_particles          = 500;
        float d_max_m              = 150.f;  // AutoDrive maximum range
        float dt_s                 = 0.10f;  // nominal timestep (s); overridden per update() call
        float process_noise_dist_m = 0.50f;  // 1-sigma position scatter per step (m)
        float process_noise_vel_ms = 0.20f;  // 1-sigma velocity scatter per step (m/s)
        float autodrive_noise_m    = 15.f;   // 1-sigma AutoDrive measurement noise (m)
        float tracker_noise_m      = 3.f;    // 1-sigma tracker measurement noise (m)
    };

    LongitudinalFusion();                    // default Config
    explicit LongitudinalFusion(Config cfg); // custom Config

    // ── Primary API ──────────────────────────────────────────────────────────
    //
    // Call once per frame, after ALL model inferences have completed (barrier).
    //
    //   autodrive_meas  : distance from AutoDrive (always pass; valid = model output valid)
    //   tracker_meas    : distance from ObjectFinder/Kalman (valid = false until tracker ported)
    //   dt_s            : actual elapsed seconds since last call; 0 → use cfg.dt_s
    //
    CIPOFusionEstimate update(const DistanceMeasurement& autodrive_meas,
                               const DistanceMeasurement& tracker_meas = {},
                               float dt_s = 0.f);

    // Hard-reset the filter.  Call on cut-in events, scene changes, or video rewinds.
    void reset();

    const Config& config() const { return cfg_; }

private:
    struct Particle {
        float distance_m;
        float velocity_ms;
        float weight;
    };

    void  init_from(float dist_m, float stddev_m);
    void  predict(float dt_s);
    void  weight_update(const DistanceMeasurement& ad, const DistanceMeasurement& tr);
    void  normalise();
    float effective_n() const;
    void  resample();

    static float gaussian_loglik(float z, float mean, float sigma);

    Config                cfg_;
    std::vector<Particle> particles_;
    bool                  initialised_ = false;
    std::mt19937          rng_;
};

}  // namespace visionpilot::fusion
