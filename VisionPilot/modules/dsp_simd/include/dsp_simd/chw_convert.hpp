#ifndef VISIONPILOT_CHW_CONVERT_HPP
#define VISIONPILOT_CHW_CONVERT_HPP

#include <opencv2/core.hpp>
#include <cstddef>

namespace visionpilot::dsp {

// BGR uint8 HWC → float CHW, ImageNet mean/std (AutoDrive / AutoSteer style).
// dst must hold at least 3*h*w floats (preferably 64-byte aligned).
void bgr_to_chw_imagenet_simd(const cv::Mat& bgr, float* dst, int w, int h);
void bgr_to_chw_imagenet_scalar(const cv::Mat& bgr, float* dst, int w, int h);

// BGR uint8 HWC → float CHW in [0,1] (AutoSpeed style).
void bgr_to_chw_01_simd(const cv::Mat& bgr, float* dst, int w, int h);
void bgr_to_chw_01_scalar(const cv::Mat& bgr, float* dst, int w, int h);

// Dispatch: SIMD if compiled/available, else scalar.
void bgr_to_chw_imagenet(const cv::Mat& bgr, float* dst, int w, int h);
void bgr_to_chw_01(const cv::Mat& bgr, float* dst, int w, int h);

}  // namespace visionpilot::dsp

#endif
