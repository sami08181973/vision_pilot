/**
 * Vision Pilot micro-benchmark: scalar vs SIMD CHW convert + memory footprint.
 *
 * Build: enabled with main VisionPilot CMake (target vp_perf_bench)
 * Run:   ./vp_perf_bench [--iters 100] [--w 1024] [--h 512]
 */
#include <dsp_simd/aligned_buffer.hpp>
#include <dsp_simd/chw_convert.hpp>
#include <dsp_simd/dsp_arch.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#if defined(__linux__)
#  include <unistd.h>
#endif

namespace {

struct Stats {
    double min_ms = 1e9, max_ms = 0, sum_ms = 0;
    int n = 0;
    void add(double ms)
    {
        min_ms = ms < min_ms ? ms : min_ms;
        max_ms = ms > max_ms ? ms : max_ms;
        sum_ms += ms;
        ++n;
    }
    double avg() const { return n ? sum_ms / n : 0; }
};

long read_rss_kb()
{
#if defined(__linux__)
    std::ifstream in("/proc/self/status");
    std::string key;
    long val = 0;
    std::string unit;
    while (in >> key) {
        if (key == "VmRSS:") {
            in >> val >> unit;
            return val;
        }
        std::string rest;
        std::getline(in, rest);
    }
#endif
    return -1;
}

template <typename Fn>
Stats bench(Fn&& fn, int iters, int warmup = 5)
{
    for (int i = 0; i < warmup; ++i) fn();
    Stats s;
    for (int i = 0; i < iters; ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        fn();
        const auto t1 = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        s.add(ms);
    }
    return s;
}

}  // namespace

int main(int argc, char** argv)
{
    int iters = 80;
    int w = 1024;
    int h = 512;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--iters") == 0 && i + 1 < argc) iters = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--w") == 0 && i + 1 < argc) w = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--h") == 0 && i + 1 < argc) h = std::atoi(argv[++i]);
    }

    cv::Mat bgr(h, w, CV_8UC3);
    cv::randu(bgr, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

    const std::size_t chw = static_cast<std::size_t>(3 * w * h);
    visionpilot::dsp::ChwArena arena(chw);
    visionpilot::dsp::AlignedVector<float> out_simd(chw);
    visionpilot::dsp::AlignedVector<float> out_scalar(chw);

    const long rss0 = read_rss_kb();

    auto s_scalar = bench([&] {
        visionpilot::dsp::bgr_to_chw_imagenet_scalar(bgr, out_scalar.data(), w, h);
    }, iters);

    auto s_simd = bench([&] {
        visionpilot::dsp::bgr_to_chw_imagenet_simd(bgr, out_simd.data(), w, h);
    }, iters);

    auto s_dispatch = bench([&] {
        visionpilot::dsp::bgr_to_chw_imagenet(bgr, arena.data(), w, h);
    }, iters);

    auto s_01 = bench([&] {
        visionpilot::dsp::bgr_to_chw_01(bgr, arena.data(), w, h);
    }, iters);

    // Legacy OpenCV path (approx. old inference.cpp)
    auto s_opencv = bench([&] {
        cv::Mat rgb, f32;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        rgb.convertTo(f32, CV_32FC3, 1.0 / 255.0);
        std::vector<cv::Mat> ch(3);
        cv::split(f32, ch);
        std::vector<float> tmp(chw);
        for (int c = 0; c < 3; ++c) {
            float* dst = tmp.data() + c * w * h;
            const float* src = reinterpret_cast<const float*>(ch[c].data);
            for (int i = 0; i < w * h; ++i)
                dst[i] = (src[i] - (c == 0 ? 0.485f : c == 1 ? 0.456f : 0.406f))
                         / (c == 0 ? 0.229f : c == 1 ? 0.224f : 0.225f);
        }
    }, iters);

    const long rss1 = read_rss_kb();
    const double speedup = s_scalar.avg() > 0 ? s_scalar.avg() / s_simd.avg() : 0;
    const double vs_cv = s_opencv.avg() > 0 ? s_opencv.avg() / s_dispatch.avg() : 0;

    const std::size_t frame_bytes = static_cast<std::size_t>(w * h * 3);
    const std::size_t chw_bytes = chw * sizeof(float);
    const std::size_t arena_bytes = arena.bytes();

    std::printf("=== Vision Pilot DSP/SIMD micro-benchmark ===\n");
    std::printf("backend     : %s\n", visionpilot::dsp::simd_backend_name());
    std::printf("frame       : %dx%d BGR8  (%zu KB)\n", w, h, frame_bytes / 1024);
    std::printf("CHW buffer  : %zu floats  (%zu KB aligned arena)\n", chw, arena_bytes / 1024);
    std::printf("iters       : %d\n", iters);
    std::printf("\n");
    std::printf("%-22s  avg_ms   min_ms   max_ms\n", "kernel");
    std::printf("%-22s  %7.3f  %7.3f  %7.3f\n", "scalar_imagenet", s_scalar.avg(), s_scalar.min_ms, s_scalar.max_ms);
    std::printf("%-22s  %7.3f  %7.3f  %7.3f\n", "simd_imagenet", s_simd.avg(), s_simd.min_ms, s_simd.max_ms);
    std::printf("%-22s  %7.3f  %7.3f  %7.3f\n", "dispatch_imagenet", s_dispatch.avg(), s_dispatch.min_ms, s_dispatch.max_ms);
    std::printf("%-22s  %7.3f  %7.3f  %7.3f\n", "dispatch_01", s_01.avg(), s_01.min_ms, s_01.max_ms);
    std::printf("%-22s  %7.3f  %7.3f  %7.3f\n", "opencv_legacy", s_opencv.avg(), s_opencv.min_ms, s_opencv.max_ms);
    std::printf("\n");
    std::printf("SIMD vs scalar speedup     : %.2fx\n", speedup);
    std::printf("dispatch vs OpenCV legacy  : %.2fx\n", vs_cv);
    std::printf("VmRSS start/end (KB)       : %ld / %ld\n", rss0, rss1);
    std::printf("working set estimate (KB)  : frame=%zu  chw=%zu  arena=%zu\n",
                frame_bytes / 1024, chw_bytes / 1024, arena_bytes / 1024);

    // Machine-readable line for report scripts
    std::printf("\nCSV,backend,scalar_ms,simd_ms,opencv_ms,speedup_simd,speedup_vs_opencv,rss_kb,chw_kb\n");
    std::printf("CSV,%s,%.4f,%.4f,%.4f,%.3f,%.3f,%ld,%zu\n",
                visionpilot::dsp::simd_backend_name(),
                s_scalar.avg(), s_simd.avg(), s_opencv.avg(),
                speedup, vs_cv, rss1, arena_bytes / 1024);
    return 0;
}
