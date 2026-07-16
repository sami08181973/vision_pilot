#include <dsp_simd/chw_convert.hpp>
#include <dsp_simd/dsp_arch.hpp>

#include <opencv2/imgproc.hpp>

#include <cstring>

#if defined(VP_SIMD_AVX2)
#  include <immintrin.h>
#elif defined(VP_SIMD_SSE2)
#  include <emmintrin.h>
#endif

#if defined(VP_SIMD_NEON)
#  include <arm_neon.h>
#endif

namespace visionpilot::dsp {
namespace {

constexpr float MEAN[3] = {0.485f, 0.456f, 0.406f};
constexpr float STD[3]  = {0.229f, 0.224f, 0.225f};
constexpr float INV255  = 1.0f / 255.0f;

}  // namespace

void bgr_to_chw_imagenet_scalar(const cv::Mat& bgr, float* dst, int w, int h)
{
    CV_Assert(bgr.type() == CV_8UC3 && bgr.cols == w && bgr.rows == h);
    const int plane = w * h;
    float* cr = dst;
    float* cg = dst + plane;
    float* cb = dst + 2 * plane;

    for (int y = 0; y < h; ++y) {
        const uint8_t* row = bgr.ptr<uint8_t>(y);
        for (int x = 0; x < w; ++x) {
            const int i = y * w + x;
            const float bf = row[x * 3 + 0] * INV255;
            const float gf = row[x * 3 + 1] * INV255;
            const float rf = row[x * 3 + 2] * INV255;
            // CHW is RGB order to match OpenCV cvtColor BGR2RGB path
            cr[i] = (rf - MEAN[0]) / STD[0];
            cg[i] = (gf - MEAN[1]) / STD[1];
            cb[i] = (bf - MEAN[2]) / STD[2];
        }
    }
}

void bgr_to_chw_01_scalar(const cv::Mat& bgr, float* dst, int w, int h)
{
    CV_Assert(bgr.type() == CV_8UC3 && bgr.cols == w && bgr.rows == h);
    const int plane = w * h;
    float* cr = dst;
    float* cg = dst + plane;
    float* cb = dst + 2 * plane;

    for (int y = 0; y < h; ++y) {
        const uint8_t* row = bgr.ptr<uint8_t>(y);
        for (int x = 0; x < w; ++x) {
            const int i = y * w + x;
            cr[i] = row[x * 3 + 2] * INV255; // R
            cg[i] = row[x * 3 + 1] * INV255; // G
            cb[i] = row[x * 3 + 0] * INV255; // B
        }
    }
}

#if defined(VP_SIMD_AVX2)

void bgr_to_chw_imagenet_simd(const cv::Mat& bgr, float* dst, int w, int h)
{
    CV_Assert(bgr.type() == CV_8UC3 && bgr.cols == w && bgr.rows == h);
    const int plane = w * h;
    float* cr = dst;
    float* cg = dst + plane;
    float* cb = dst + 2 * plane;

    const __m256 vinv = _mm256_set1_ps(INV255);
    const __m256 vm0 = _mm256_set1_ps(MEAN[0]);
    const __m256 vm1 = _mm256_set1_ps(MEAN[1]);
    const __m256 vm2 = _mm256_set1_ps(MEAN[2]);
    const __m256 vs0 = _mm256_set1_ps(1.0f / STD[0]);
    const __m256 vs1 = _mm256_set1_ps(1.0f / STD[1]);
    const __m256 vs2 = _mm256_set1_ps(1.0f / STD[2]);

    for (int y = 0; y < h; ++y) {
        const uint8_t* row = bgr.ptr<uint8_t>(y);
        int x = 0;
        for (; x + 8 <= w; x += 8) {
            alignas(32) float rf[8], gf[8], bf[8];
            for (int k = 0; k < 8; ++k) {
                bf[k] = row[(x + k) * 3 + 0];
                gf[k] = row[(x + k) * 3 + 1];
                rf[k] = row[(x + k) * 3 + 2];
            }
            __m256 r = _mm256_mul_ps(_mm256_load_ps(rf), vinv);
            __m256 g = _mm256_mul_ps(_mm256_load_ps(gf), vinv);
            __m256 b = _mm256_mul_ps(_mm256_load_ps(bf), vinv);
            r = _mm256_mul_ps(_mm256_sub_ps(r, vm0), vs0);
            g = _mm256_mul_ps(_mm256_sub_ps(g, vm1), vs1);
            b = _mm256_mul_ps(_mm256_sub_ps(b, vm2), vs2);
            const int i = y * w + x;
            _mm256_storeu_ps(cr + i, r);
            _mm256_storeu_ps(cg + i, g);
            _mm256_storeu_ps(cb + i, b);
        }
        for (; x < w; ++x) {
            const int i = y * w + x;
            const float bv = row[x * 3 + 0] * INV255;
            const float gv = row[x * 3 + 1] * INV255;
            const float rv = row[x * 3 + 2] * INV255;
            cr[i] = (rv - MEAN[0]) / STD[0];
            cg[i] = (gv - MEAN[1]) / STD[1];
            cb[i] = (bv - MEAN[2]) / STD[2];
        }
    }
}

void bgr_to_chw_01_simd(const cv::Mat& bgr, float* dst, int w, int h)
{
    CV_Assert(bgr.type() == CV_8UC3 && bgr.cols == w && bgr.rows == h);
    const int plane = w * h;
    float* cr = dst;
    float* cg = dst + plane;
    float* cb = dst + 2 * plane;
    const __m256 vinv = _mm256_set1_ps(INV255);

    for (int y = 0; y < h; ++y) {
        const uint8_t* row = bgr.ptr<uint8_t>(y);
        int x = 0;
        for (; x + 8 <= w; x += 8) {
            alignas(32) float rf[8], gf[8], bf[8];
            for (int k = 0; k < 8; ++k) {
                bf[k] = row[(x + k) * 3 + 0];
                gf[k] = row[(x + k) * 3 + 1];
                rf[k] = row[(x + k) * 3 + 2];
            }
            const int i = y * w + x;
            _mm256_storeu_ps(cr + i, _mm256_mul_ps(_mm256_load_ps(rf), vinv));
            _mm256_storeu_ps(cg + i, _mm256_mul_ps(_mm256_load_ps(gf), vinv));
            _mm256_storeu_ps(cb + i, _mm256_mul_ps(_mm256_load_ps(bf), vinv));
        }
        for (; x < w; ++x) {
            const int i = y * w + x;
            cr[i] = row[x * 3 + 2] * INV255;
            cg[i] = row[x * 3 + 1] * INV255;
            cb[i] = row[x * 3 + 0] * INV255;
        }
    }
}

#elif defined(VP_SIMD_SSE2)

void bgr_to_chw_imagenet_simd(const cv::Mat& bgr, float* dst, int w, int h)
{
    CV_Assert(bgr.type() == CV_8UC3 && bgr.cols == w && bgr.rows == h);
    const int plane = w * h;
    float* cr = dst;
    float* cg = dst + plane;
    float* cb = dst + 2 * plane;

    const __m128 vinv = _mm_set1_ps(INV255);
    const __m128 vm0 = _mm_set1_ps(MEAN[0]);
    const __m128 vm1 = _mm_set1_ps(MEAN[1]);
    const __m128 vm2 = _mm_set1_ps(MEAN[2]);
    const __m128 vs0 = _mm_set1_ps(1.0f / STD[0]);
    const __m128 vs1 = _mm_set1_ps(1.0f / STD[1]);
    const __m128 vs2 = _mm_set1_ps(1.0f / STD[2]);

    for (int y = 0; y < h; ++y) {
        const uint8_t* row = bgr.ptr<uint8_t>(y);
        int x = 0;
        for (; x + 4 <= w; x += 4) {
            alignas(16) float rf[4], gf[4], bf[4];
            for (int k = 0; k < 4; ++k) {
                bf[k] = static_cast<float>(row[(x + k) * 3 + 0]);
                gf[k] = static_cast<float>(row[(x + k) * 3 + 1]);
                rf[k] = static_cast<float>(row[(x + k) * 3 + 2]);
            }
            __m128 r = _mm_mul_ps(_mm_load_ps(rf), vinv);
            __m128 g = _mm_mul_ps(_mm_load_ps(gf), vinv);
            __m128 b = _mm_mul_ps(_mm_load_ps(bf), vinv);
            r = _mm_mul_ps(_mm_sub_ps(r, vm0), vs0);
            g = _mm_mul_ps(_mm_sub_ps(g, vm1), vs1);
            b = _mm_mul_ps(_mm_sub_ps(b, vm2), vs2);
            const int i = y * w + x;
            _mm_storeu_ps(cr + i, r);
            _mm_storeu_ps(cg + i, g);
            _mm_storeu_ps(cb + i, b);
        }
        for (; x < w; ++x) {
            const int i = y * w + x;
            const float bv = row[x * 3 + 0] * INV255;
            const float gv = row[x * 3 + 1] * INV255;
            const float rv = row[x * 3 + 2] * INV255;
            cr[i] = (rv - MEAN[0]) / STD[0];
            cg[i] = (gv - MEAN[1]) / STD[1];
            cb[i] = (bv - MEAN[2]) / STD[2];
        }
    }
}

void bgr_to_chw_01_simd(const cv::Mat& bgr, float* dst, int w, int h)
{
    CV_Assert(bgr.type() == CV_8UC3 && bgr.cols == w && bgr.rows == h);
    const int plane = w * h;
    float* cr = dst;
    float* cg = dst + plane;
    float* cb = dst + 2 * plane;
    const __m128 vinv = _mm_set1_ps(INV255);

    for (int y = 0; y < h; ++y) {
        const uint8_t* row = bgr.ptr<uint8_t>(y);
        int x = 0;
        for (; x + 4 <= w; x += 4) {
            alignas(16) float rf[4], gf[4], bf[4];
            for (int k = 0; k < 4; ++k) {
                bf[k] = static_cast<float>(row[(x + k) * 3 + 0]);
                gf[k] = static_cast<float>(row[(x + k) * 3 + 1]);
                rf[k] = static_cast<float>(row[(x + k) * 3 + 2]);
            }
            const int i = y * w + x;
            _mm_storeu_ps(cr + i, _mm_mul_ps(_mm_load_ps(rf), vinv));
            _mm_storeu_ps(cg + i, _mm_mul_ps(_mm_load_ps(gf), vinv));
            _mm_storeu_ps(cb + i, _mm_mul_ps(_mm_load_ps(bf), vinv));
        }
        for (; x < w; ++x) {
            const int i = y * w + x;
            cr[i] = row[x * 3 + 2] * INV255;
            cg[i] = row[x * 3 + 1] * INV255;
            cb[i] = row[x * 3 + 0] * INV255;
        }
    }
}

#elif defined(VP_SIMD_NEON)

void bgr_to_chw_imagenet_simd(const cv::Mat& bgr, float* dst, int w, int h)
{
    CV_Assert(bgr.type() == CV_8UC3 && bgr.cols == w && bgr.rows == h);
    const int plane = w * h;
    float* cr = dst;
    float* cg = dst + plane;
    float* cb = dst + 2 * plane;
    const float32x4_t vinv = vdupq_n_f32(INV255);
    const float32x4_t vm0 = vdupq_n_f32(MEAN[0]);
    const float32x4_t vm1 = vdupq_n_f32(MEAN[1]);
    const float32x4_t vm2 = vdupq_n_f32(MEAN[2]);
    const float32x4_t vs0 = vdupq_n_f32(1.0f / STD[0]);
    const float32x4_t vs1 = vdupq_n_f32(1.0f / STD[1]);
    const float32x4_t vs2 = vdupq_n_f32(1.0f / STD[2]);

    for (int y = 0; y < h; ++y) {
        const uint8_t* row = bgr.ptr<uint8_t>(y);
        int x = 0;
        for (; x + 4 <= w; x += 4) {
            float rf[4], gf[4], bf[4];
            for (int k = 0; k < 4; ++k) {
                bf[k] = row[(x + k) * 3 + 0];
                gf[k] = row[(x + k) * 3 + 1];
                rf[k] = row[(x + k) * 3 + 2];
            }
            float32x4_t r = vmulq_f32(vld1q_f32(rf), vinv);
            float32x4_t g = vmulq_f32(vld1q_f32(gf), vinv);
            float32x4_t b = vmulq_f32(vld1q_f32(bf), vinv);
            r = vmulq_f32(vsubq_f32(r, vm0), vs0);
            g = vmulq_f32(vsubq_f32(g, vm1), vs1);
            b = vmulq_f32(vsubq_f32(b, vm2), vs2);
            const int i = y * w + x;
            vst1q_f32(cr + i, r);
            vst1q_f32(cg + i, g);
            vst1q_f32(cb + i, b);
        }
        for (; x < w; ++x) {
            const int i = y * w + x;
            const float bv = row[x * 3 + 0] * INV255;
            const float gv = row[x * 3 + 1] * INV255;
            const float rv = row[x * 3 + 2] * INV255;
            cr[i] = (rv - MEAN[0]) / STD[0];
            cg[i] = (gv - MEAN[1]) / STD[1];
            cb[i] = (bv - MEAN[2]) / STD[2];
        }
    }
}

void bgr_to_chw_01_simd(const cv::Mat& bgr, float* dst, int w, int h)
{
    // Reuse imagenet path structure without mean/std
    CV_Assert(bgr.type() == CV_8UC3 && bgr.cols == w && bgr.rows == h);
    const int plane = w * h;
    float* cr = dst;
    float* cg = dst + plane;
    float* cb = dst + 2 * plane;
    const float32x4_t vinv = vdupq_n_f32(INV255);

    for (int y = 0; y < h; ++y) {
        const uint8_t* row = bgr.ptr<uint8_t>(y);
        int x = 0;
        for (; x + 4 <= w; x += 4) {
            float rf[4], gf[4], bf[4];
            for (int k = 0; k < 4; ++k) {
                bf[k] = row[(x + k) * 3 + 0];
                gf[k] = row[(x + k) * 3 + 1];
                rf[k] = row[(x + k) * 3 + 2];
            }
            const int i = y * w + x;
            vst1q_f32(cr + i, vmulq_f32(vld1q_f32(rf), vinv));
            vst1q_f32(cg + i, vmulq_f32(vld1q_f32(gf), vinv));
            vst1q_f32(cb + i, vmulq_f32(vld1q_f32(bf), vinv));
        }
        for (; x < w; ++x) {
            const int i = y * w + x;
            cr[i] = row[x * 3 + 2] * INV255;
            cg[i] = row[x * 3 + 1] * INV255;
            cb[i] = row[x * 3 + 0] * INV255;
        }
    }
}

#else

// SSE2 or no SIMD: use scalar kernels (still benefits from aligned arena + fewer allocs)
void bgr_to_chw_imagenet_simd(const cv::Mat& bgr, float* dst, int w, int h)
{
    bgr_to_chw_imagenet_scalar(bgr, dst, w, h);
}

void bgr_to_chw_01_simd(const cv::Mat& bgr, float* dst, int w, int h)
{
    bgr_to_chw_01_scalar(bgr, dst, w, h);
}

#endif

void bgr_to_chw_imagenet(const cv::Mat& bgr, float* dst, int w, int h)
{
#if VP_SIMD_ENABLED
    bgr_to_chw_imagenet_simd(bgr, dst, w, h);
#else
    bgr_to_chw_imagenet_scalar(bgr, dst, w, h);
#endif
}

void bgr_to_chw_01(const cv::Mat& bgr, float* dst, int w, int h)
{
#if VP_SIMD_ENABLED
    bgr_to_chw_01_simd(bgr, dst, w, h);
#else
    bgr_to_chw_01_scalar(bgr, dst, w, h);
#endif
}

}  // namespace visionpilot::dsp
