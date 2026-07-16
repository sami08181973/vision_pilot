#ifndef VISIONPILOT_DSP_ARCH_HPP
#define VISIONPILOT_DSP_ARCH_HPP

// Architecture / SIMD capability detection for Vision Pilot DSP paths.
// Hosts: x86 SSE2/AVX2, ARM NEON. Scaffolds: TI C66x, Qualcomm Hexagon.

#if defined(__AVX2__)
#  define VP_SIMD_AVX2 1
#elif defined(__SSE2__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
#  define VP_SIMD_SSE2 1
#endif

#if defined(__ARM_NEON) || defined(__ARM_NEON__) || defined(__aarch64__)
#  define VP_SIMD_NEON 1
#endif

#if defined(_TMS320C6600) || defined(__C66__) || defined(__TMS320C6X__)
#  define VP_SIMD_C66X 1
#endif

#if defined(__hexagon__) || defined(__HEXAGON_ARCH__)
#  define VP_SIMD_HEXAGON 1
#endif

#if defined(VP_SIMD_AVX2) || defined(VP_SIMD_SSE2) || defined(VP_SIMD_NEON) \
    || defined(VP_SIMD_C66X) || defined(VP_SIMD_HEXAGON)
#  define VP_SIMD_ENABLED 1
#else
#  define VP_SIMD_ENABLED 0
#endif

namespace visionpilot::dsp {

inline const char* simd_backend_name()
{
#if defined(VP_SIMD_AVX2)
    return "AVX2";
#elif defined(VP_SIMD_SSE2)
    return "SSE2";
#elif defined(VP_SIMD_NEON)
    return "NEON";
#elif defined(VP_SIMD_C66X)
    return "TI_C66x";
#elif defined(VP_SIMD_HEXAGON)
    return "Hexagon_HVX";
#else
    return "scalar";
#endif
}

}  // namespace visionpilot::dsp

#endif
