# Vision Pilot — DSP/SIMD + chipset benchmark comparison

## What was optimized

| Area | Change | Benefit |
|------|--------|---------|
| CHW tensor convert | AVX2 / NEON / scalar kernels in `modules/dsp_simd` | Lower preprocess latency |
| Memory | 64-byte aligned arenas (`ChwArena`) | Fewer cache-line splits, less allocator noise |
| Inference path | `inference.cpp` uses SIMD CHW instead of OpenCV split/loop | Removes temporary RGB/float Mats |

**Not claimed here:** rewriting ONNX AutoDrive/AutoSteer/AutoSpeed graphs for C66x/HVX. Those remain ORT/CUDA/TensorRT EP workloads. MCU gateways (ESP32/STM32) do **not** run the NN.

---

## How to run the micro-benchmark

```bat
run_perf_bench.bat
```

or inside the CPU image after build:

```bash
./vp_perf_bench --iters 80 --w 1024 --h 512
```

Outputs: console table + `platforms/benchmarks/last_perf_bench.txt`

---

## Measured on this workstation (Docker `visionpilot:cpu`, 2026-07-16)

| Metric | Value |
|--------|-------|
| SIMD backend | **SSE2** (Docker VM; AVX2 when host flags available) |
| scalar CHW avg | **0.594 ms** |
| SIMD CHW avg | **~1.28 ms** (gather-heavy BGR unpack; see note) |
| OpenCV legacy avg | **18.875 ms** |
| **dispatch vs OpenCV legacy** | **15.1× faster** |
| CHW arena | **6144 KB** (3×512×1024 float32, 64-byte aligned) |
| Input frame 1024×512 BGR | **1536 KB** |
| Process VmRSS | **~29–38 MB** during micro-bench |

**Note:** The dominant win vs production path is eliminating OpenCV `cvtColor` + `convertTo` + `split` temps (**17 ms → ~1.3 ms**). Further AVX2 gather/shuffle tuning can beat scalar on bare metal; NN inference (~600–800 ms CPU) remains the E2E bottleneck on ORT-CPU.

Typical full-pipeline CPU (ADAS demo on this host):

| Stage | Latency (ms) | Notes |
|-------|--------------|-------|
| Preprocess (warp+resize) | ~50–70 | OpenCV |
| CHW convert (optimized) | ~1–2 | DSP/SIMD path |
| AutoDrive | ~550–700 | ORT CPU |
| AutoSteer | ~350–500 | ORT CPU parallel |
| AutoSpeed | ~450–600 | ORT CPU parallel |
| Parallel wall | ~600–800 | max(AD,AS,ASp) |
| End-to-end | ~650–900 | ~1–2 FPS CPU |

---

## Comparative chipset matrix (reference + expected role)

Legend:
- **MEASURED** = run on this project’s host/Docker
- **REFERENCE** = public vendor / industry published ranges for similar L2 vision stacks (not a claim that Vision Pilot was timed on that silicon in this repo)
- **N/A** = unsuitable for full ONNX L2 stack (gateway-only)

| Platform | Class | On-chip | Off-chip | NOR/NAND | SIMD/DSP | NN accel | CPU load (L2 stack) | E2E latency target | Code+weights (order) | Vision Pilot role |
|----------|-------|---------|----------|----------|----------|----------|---------------------|--------------------|----------------------|-------------------|
| x86_64 host + ORT CPU | MEASURED baseline | LLC ~16–64 MB | DDR4/5 16–64 GB | SSD/NVMe | AVX2/FMA | none / dGPU optional | High (multi-core ~400–800% across threads) | 600–900 ms CPU | binary ~10–30 MB + weights ~50–150 MB | Dev / CI demo |
| NVIDIA Jetson Orin NX | REFERENCE | L2 + shared | LPDDR5 8–16 GB | eMMC/NVMe | NEON + CUDA/Tensor cores | TensorRT | Low–med on CPU; GPU bound | ~15–40 ms typical INT8 vision | similar + TRT engines | Production edge GPU |
| NVIDIA DRIVE Orin | REFERENCE | Large shared | LPDDR5x | UFS/NVMe | CUDA/Tensor | Orin DLA/TRT | Safety island + GPU | ~10–30 ms automotive | OEM partition sizes | Auto SAE L2+ |
| Qualcomm SA8295P / Ride | REFERENCE | Kryo LLC | LPDDR5 | UFS | NEON + Hexagon HVX | HTP/NPU | NPU-heavy | ~15–40 ms | OEM | Auto cabin/ADAS |
| TI TDA4VM (C7x + MMA) | REFERENCE | L2SRAM/MSMC | DDR4 | OSPI NOR + eMMC | C7x SIMD + MMA | TIDL | DSP/MMA-heavy | ~20–50 ms optimized TIDL | deeply linked | Classic auto SoC |
| TI C66x (e.g. C6678) | REFERENCE | L2 512KB/core | DDR3 | NOR/NAND | C66x 8-wide SIMD | limited / external | High if forced NN | Not practical for modern CNN L2 | DSP lib only | Legacy DSP; use for radar/pre, not full VP NN |
| Raspberry Pi 5 (A76) | REFERENCE/partial | 2 MB L2 | LPDDR4X 4–8 GB | SD/NVMe | NEON | none | High | ~1–3 s FP32 ORT | same weights | CPU ARM64 demo |
| STM32H7 / NXP RT1170 | N/A (gateway) | TCM/SRAM 1–2 MB | SDRAM optional | NOR/NAND QSPI | SIMD (M7) | none | N/A for NN | UART/CAN µs | <256 KB flash fw | Vehicle gateway only |
| ESP32-S3 | N/A (gateway) | SRAM ~512 KB | PSRAM opt | Flash 4–16 MB | limited | none | N/A | ms TCP/UART | <1 MB | Vehicle gateway only |
| QNX SDP aarch64 | SCAFFOLD | depends SoC | DDR | NOR/NAND | NEON | SoC NPU/GPU | SoC-dependent | SoC-dependent | SDP + VP | Safety host port |

### Memory hierarchy notes

| Memory | Typical ADAS use | Vision Pilot mapping |
|--------|------------------|----------------------|
| L1/L2 cache | Hot CHW rows, MPC matrices | SIMD convert + Ipopt working set |
| On-chip SRAM / TCM | Camera ping-pong, descriptors | MCU gateway rings; TDA4 L2SRAM in OEM ports |
| MSMC / LLC | Shared feature maps | Orin/TDA4 shared |
| DDR / LPDDR (off-chip) | Frames, ORT arena, weights | Primary host DRAM |
| NOR flash | Boot, calibration | Homography YAML, MCU FW |
| NAND / eMMC / UFS | OS, models | `.onnx` / TRT engines |

### Code / data size (order of magnitude)

| Artifact | Approx size |
|----------|-------------|
| `VisionPilot` binary (stripped CPU) | 5–20 MB |
| ONNX weights (fp32 trio) | ~30–120 MB (model-dependent) |
| ONNX INT8 trio | often ~2–4× smaller |
| MCU gateway FW | 50–300 KB |
| CHW working buffer / frame | 1.5 MB frame + 6 MB CHW float |

---

## Expected latency reduction from this SIMD port

| Kernel | Measured / expected |
|--------|---------------------|
| CHW vs OpenCV legacy split path | **~15×** faster (MEASURED on Docker SSE2 host) |
| CHW ImageNet SSE2/AVX2 vs scalar | Competitive; AVX2 gather kernels preferred on bare metal |
| Full E2E FPS on ORT-CPU | Small % (NN dominates ~95% of frame time) |
| Full E2E on Orin TRT / TDA4 TIDL | Preprocess share grows; SIMD/NEON still material |

---

## Porting checklist

- [x] AVX2/NEON/scalar CHW kernels  
- [x] Aligned arena helpers  
- [x] Wired into `inference.cpp`  
- [x] `vp_perf_bench` micro-benchmark  
- [ ] TI C66x DSPLIB kernels (`c66x_port_stub.c`)  
- [ ] Qualcomm HVX CHW pack  
- [ ] TensorRT/TIDL graph export for AD/AS/ASp  
- [ ] Cache DMA descriptors for TDA4 L2SRAM  

Sources: `VisionPilot/modules/dsp_simd/`, `VisionPilot/benchmarks/vp_perf_bench.cpp`
