#include "fusion/longitudinal_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace visionpilot::fusion {

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

LongitudinalFusion::LongitudinalFusion()
    : LongitudinalFusion(Config{})
{}

LongitudinalFusion::LongitudinalFusion(Config cfg)
    : cfg_(cfg)
    , rng_(std::random_device{}())
{
    if (cfg_.n_particles < 10) {
        throw std::invalid_argument("LongitudinalFusion: n_particles must be >= 10");
    }
    particles_.reserve(static_cast<std::size_t>(cfg_.n_particles));
}

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────

void LongitudinalFusion::reset()
{
    particles_.clear();
    initialised_ = false;
}

CIPOFusionEstimate LongitudinalFusion::update(
    const DistanceMeasurement& autodrive_meas,
    const DistanceMeasurement& tracker_meas,
    float dt_s)
{
    const float dt = (dt_s > 1e-6f) ? dt_s : cfg_.dt_s;

    // Wait for the first valid measurement before allocating particles.
    if (!initialised_) {
        if (!autodrive_meas.valid) {
            return {};
        }
        init_from(autodrive_meas.distance_m, autodrive_meas.stddev_m);
        initialised_ = true;
        // On the very first frame we skip the predict step — nothing to propagate yet.
    } else {
        predict(dt);
    }

    // Apply Gaussian likelihoods from all valid measurement sources.
    weight_update(autodrive_meas, tracker_meas);
    normalise();

    // Systematic resampling to fight weight degeneracy.
    if (effective_n() < 0.5f * static_cast<float>(cfg_.n_particles)) {
        resample();
    }

    // Compute posterior statistics (weighted mean and variance).
    float mean_d = 0.f, mean_v = 0.f;
    for (const auto& p : particles_) {
        mean_d += p.weight * p.distance_m;
        mean_v += p.weight * p.velocity_ms;
    }

    float var_d = 0.f, var_v = 0.f;
    for (const auto& p : particles_) {
        const float dd = p.distance_m  - mean_d;
        const float dv = p.velocity_ms - mean_v;
        var_d += p.weight * dd * dd;
        var_v += p.weight * dv * dv;
    }

    CIPOFusionEstimate est;
    est.valid              = true;
    est.distance_m         = mean_d;
    est.velocity_ms        = mean_v;
    est.distance_stddev_m  = std::sqrt(std::max(0.f, var_d));
    est.velocity_stddev_ms = std::sqrt(std::max(0.f, var_v));
    return est;
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

void LongitudinalFusion::init_from(float dist_m, float stddev_m)
{
    particles_.resize(static_cast<std::size_t>(cfg_.n_particles));

    std::normal_distribution<float> d_dist(dist_m, stddev_m);
    // Unknown initial velocity — spread particles over a reasonable range.
    std::normal_distribution<float> v_dist(0.f, 2.f);

    const float w0 = 1.f / static_cast<float>(cfg_.n_particles);
    for (auto& p : particles_) {
        p.distance_m  = std::clamp(d_dist(rng_), 0.f, cfg_.d_max_m);
        p.velocity_ms = v_dist(rng_);
        p.weight      = w0;
    }
}

void LongitudinalFusion::predict(float dt_s)
{
    std::normal_distribution<float> n_d(0.f, cfg_.process_noise_dist_m);
    std::normal_distribution<float> n_v(0.f, cfg_.process_noise_vel_ms);

    for (auto& p : particles_) {
        // Constant-velocity model with additive noise.
        p.distance_m  = std::clamp(
            p.distance_m + p.velocity_ms * dt_s + n_d(rng_), 0.f, cfg_.d_max_m);
        p.velocity_ms = p.velocity_ms + n_v(rng_);
    }
}

// Unnormalised log-likelihood of a scalar Gaussian measurement.
float LongitudinalFusion::gaussian_loglik(float z, float mean, float sigma)
{
    const float diff = z - mean;
    return -0.5f * (diff / sigma) * (diff / sigma);
}

void LongitudinalFusion::weight_update(
    const DistanceMeasurement& ad, const DistanceMeasurement& tr)
{
    // Work in log-space to avoid underflow when multiple likelihoods are multiplied.
    for (auto& p : particles_) {
        float log_w = 0.f;

        if (ad.valid) {
            log_w += gaussian_loglik(ad.distance_m, p.distance_m, ad.stddev_m);
        }
        if (tr.valid) {
            log_w += gaussian_loglik(tr.distance_m, p.distance_m, tr.stddev_m);
        }

        // Multiply the accumulated likelihood into the existing particle weight.
        p.weight *= std::exp(log_w);
        // Guard against exact zero (prevents division by zero in normalise()).
        p.weight  = std::max(p.weight, 1e-12f);
    }
}

void LongitudinalFusion::normalise()
{
    float sum = 0.f;
    for (const auto& p : particles_) sum += p.weight;

    if (sum < 1e-12f) {
        // Complete weight collapse — fall back to uniform weights so the filter
        // can recover rather than being stuck at zero forever.
        const float w0 = 1.f / static_cast<float>(cfg_.n_particles);
        for (auto& p : particles_) p.weight = w0;
    } else {
        for (auto& p : particles_) p.weight /= sum;
    }
}

float LongitudinalFusion::effective_n() const
{
    float sum_sq = 0.f;
    for (const auto& p : particles_) sum_sq += p.weight * p.weight;
    return 1.f / (sum_sq + 1e-12f);
}

void LongitudinalFusion::resample()
{
    const int N = static_cast<int>(particles_.size());
    if (N == 0) return;

    // Build cumulative sum of weights.
    std::vector<float> cumsum(static_cast<std::size_t>(N));
    cumsum[0] = particles_[0].weight;
    for (int i = 1; i < N; ++i) {
        cumsum[static_cast<std::size_t>(i)] =
            cumsum[static_cast<std::size_t>(i - 1)] + particles_[static_cast<std::size_t>(i)].weight;
    }

    // Systematic / low-variance resampling (O(N), unbiased, low variance).
    std::vector<Particle> new_particles;
    new_particles.reserve(static_cast<std::size_t>(N));

    std::uniform_real_distribution<float> u_dist(0.f, 1.f / static_cast<float>(N));
    const float u   = u_dist(rng_);
    const float w0  = 1.f / static_cast<float>(N);
    int         j   = 0;

    for (int i = 0; i < N; ++i) {
        const float threshold = u + static_cast<float>(i) / static_cast<float>(N);
        while (j < N - 1 &&
               cumsum[static_cast<std::size_t>(j)] < threshold) {
            ++j;
        }
        new_particles.push_back({
            particles_[static_cast<std::size_t>(j)].distance_m,
            particles_[static_cast<std::size_t>(j)].velocity_ms,
            w0
        });
    }

    particles_ = std::move(new_particles);
}

}  // namespace visionpilot::fusion
