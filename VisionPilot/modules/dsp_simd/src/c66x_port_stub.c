/*
 * TI C66x / C7x port scaffold for Vision Pilot CHW kernels.
 *
 * Build with TI CGT (cl6x) after mapping:
 *   - DSP_FFT / MULSP / DSPF_sp_dotp for vector math
 *   - EDMA3 for camera DMA into L2SRAM
 *   - MSMC / DDR3 placement via linker CMD file
 *
 * This file documents the ABI expected by chw_convert when
 * VP_SIMD_C66X is defined. Full C66x intrinsics require TI SDK.
 */
#if defined(_TMS320C6600) || defined(__C66__)

#include <dsp_simd/chw_convert.hpp>
/* Implement with TI DSPLIB / intrinsic vector packs when porting. */

#endif
