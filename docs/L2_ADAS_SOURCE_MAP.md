# Vision Pilot L2 ADAS — Complete Source Map

Everything required to **build, run, and demonstrate** entry L2 ADAS (ACC / FCW / AEB / LKAS / LDW / ISA / Autopilot) with the hybrid E2E stack, virtual hardware, MCU gateways, and cloud/CI scaffolding.

**One-click run (Windows):** `run_adas_demo.bat`  
**Public fork:** https://github.com/sami08181973/vision_pilot

---

## 1. Main application (pipeline orchestration)

| File | Role |
|------|------|
| `VisionPilot/app/vision_pilot.cpp` | Main loop: camera → preprocess → ONNX → fusion → plan → actuators; ADAS feature status logging |
| `VisionPilot/CMakeLists.txt` | Root build |
| `VisionPilot/package.xml` | ROS package metadata |

---

## 2. L2 feature logic (ACC / FCW / AEB / LKAS / LDW / ISA / Autopilot)

| Feature | Source | Role |
|---------|--------|------|
| Warning types | `VisionPilot/modules/common/include/common/types.hpp` | `FCW`, `AEB`, `LLDW`, `RLDW` + `Plan` |
| FCW / AEB / LDW | `VisionPilot/modules/safety_guardian/planning/src/planning.cpp` | Threshold-based warnings |
| ACC (longitudinal) | `VisionPilot/modules/safety_guardian/planning/src/longitudinal_planning.cpp` | IDM accel vs limit / CIPO |
| LKAS (lateral MPC) | `VisionPilot/modules/safety_guardian/planning/src/lateral_planning.cpp` | Ipopt/CppAD steering |
| Planner API | `VisionPilot/modules/safety_guardian/planning/include/planning/*.hpp` | Headers |
| ACC/LKAS/ISA/Autopilot flags | `VisionPilot/app/vision_pilot.cpp` | Derived ON/OFF / ENGAGED logging |

---

## 3. Hybrid End-to-End AI (AutoDrive / AutoSteer / AutoSpeed)

| Component | Source |
|-----------|--------|
| AutoDrive | `VisionPilot/modules/models/src/auto_drive.cpp`, `include/models/auto_drive.hpp` |
| AutoSteer | `VisionPilot/modules/models/src/auto_steer.cpp`, `include/models/auto_steer.hpp` |
| AutoSpeed | `VisionPilot/modules/models/src/auto_speed.cpp`, `include/models/auto_speed.hpp` |
| Inference pipeline | `VisionPilot/modules/models/src/inference.cpp`, `include/models/inference.hpp` |
| ONNX Runtime engine | `VisionPilot/modules/engine/src/onnx_engine.cpp` |
| TensorRT helper | `VisionPilot/modules/models/src/trt_engine.cpp` |
| Model weights | `VisionPilot/modules/models/weights/*.onnx` (fp32 + int8) |

---

## 4. Sensing (camera + vehicle)

| Component | Source |
|-----------|--------|
| Live camera (V4L2) | `VisionPilot/modules/sensing/camera_interface/src/v4l2_camera_interface.cpp` |
| Video file camera | `VisionPilot/modules/sensing/camera_interface/src/file_interface.cpp` |
| Ego speed file | `VisionPilot/modules/sensing/vehicle_interface/src/file_interface.cpp` |
| Vehicle TCP gateway | `VisionPilot/modules/sensing/vehicle_interface/src/can_interface.cpp` |
| Interfaces | `VisionPilot/modules/sensing/*/include/**/*.hpp` |
| Image preprocess | `VisionPilot/modules/sensing/image_preprocessing/src/image_preprocessor.cpp` |
| Homography | `VisionPilot/config/H.yaml`, `VisionPilot/scripts/find_homography_C_matrix.py` |
| Calibration guide | `Calibration/` |
| Sample DBC | `VisionPilot/config/vehicle.dbc` |
| Gateway protocol | `platforms/protocol/vp_vehicle_gateway.md` |

Optional ROS2:

- `VisionPilot/modules/middleware_interfaces/ros2_interface/camera_ros2_interface/`
- `VisionPilot/modules/middleware_interfaces/ros2_interface/vehicle_ros2_interface/`

---

## 5. Fusion (safety guardian)

| Component | Source |
|-----------|--------|
| Longitudinal (CIPO) | `VisionPilot/modules/safety_guardian/fusion/src/longitudinal_fusion.cpp` |
| Lateral (CTE / yaw / κ) | `VisionPilot/modules/safety_guardian/fusion/src/lateral_fusion.cpp` |
| Headers | `VisionPilot/modules/safety_guardian/fusion/include/fusion/*.hpp` |

---

## 6. Config / logging / common

| Component | Source |
|-----------|--------|
| Config parser | `VisionPilot/modules/config/src/vision_pilot_config.cpp` |
| Demo configs | `VisionPilot/config/vision_pilot.conf.demo`, `vision_pilot_test.conf.demo` |
| CPU example | `VisionPilot/config/vision_pilot.conf.cpu.example` |
| Logging | `VisionPilot/modules/logging/` |
| Utils | `VisionPilot/modules/common/` |

---

## 7. Visualization

| Component | Source |
|-----------|--------|
| HUD / overlays | `VisionPilot/modules/visualization/src/visualization.cpp` |
| Local window | `VisionPilot/modules/visualization/src/local_display.cpp` |
| Headless (Docker) | `VisionPilot/modules/visualization/src/headless_display.cpp` |
| WebRTC (optional) | `VisionPilot/modules/visualization/src/webrtc_stream.cpp` |

---

## 8. Build, Docker, one-click demo

| Component | Source |
|-----------|--------|
| One-click Windows | `run_adas_demo.bat`, `run_adas_demo.ps1` |
| Demo compose | `VisionPilot/docker/docker-compose.demo.yml` |
| Stack compose | `VisionPilot/docker/docker-compose.yml` |
| Dockerfiles | `VisionPilot/docker/Dockerfile`, `Dockerfile.cpu` |
| Build/run scripts | `VisionPilot/docker/build.sh`, `run.sh` |
| Deployment guide | `docs/DEPLOYMENT.md` |

---

## 9. Virtual hardware (no physical camera / MCU)

| Component | Source |
|-----------|--------|
| Synthetic camera + speed | `platforms/virtual_hw/generate_demo_assets.py` |
| Virtual ECU | `platforms/virtual_hw/virtual_ecu.py` |
| Helpers | `platforms/virtual_hw/run_demo.ps1`, `run_demo.sh` |
| Docs | `platforms/virtual_hw/README.md` |

---

## 10. MCU / RTOS vehicle gateways

| Platform | Source |
|----------|--------|
| Arduino | `platforms/arduino/vision_pilot_gateway/vision_pilot_gateway.ino` |
| ESP32 | `platforms/esp32/vision_pilot_gateway/main/main.c` (+ `CMakeLists.txt`) |
| Zephyr (STM32 / NXP / TI) | `platforms/zephyr/apps/vp_gateway/src/main.c`, `prj.conf`, `CMakeLists.txt` |
| Platform index | `platforms/README.md` |

---

## 11. Host ports (RPi5 / QNX SDP)

| Platform | Source |
|----------|--------|
| Raspberry Pi 5 | `platforms/raspberry_pi5/README.md` |
| QNX SDP toolchain | `platforms/qnx_sdp/cmake/qnx.toolchain.cmake`, `platforms/qnx_sdp/README.md` |

---

## 12. CI / multi-cloud

| Component | Source |
|-----------|--------|
| Jenkins | `Jenkinsfile`, `infra/jenkins/Jenkinsfile`, `infra/jenkins/README.md` |
| Terraform AWS | `infra/terraform/aws/main.tf` |
| Terraform GCP | `infra/terraform/gcp/main.tf` |
| Terraform Azure | `infra/terraform/azure/main.tf` |
| CI cloud-init | `infra/terraform/modules/cloud_init_ci.sh` |
| Terraform docs | `infra/terraform/README.md` |

---

## 13. Tests

| Area | Source |
|------|--------|
| App | `VisionPilot/tests/app/test_vision_pilot.cpp` |
| Planning | `VisionPilot/tests/safety_guardian/planning/test_*.cpp` |
| Preprocess | `VisionPilot/tests/sensing/image_preprocessing/test_image_preprocessor.cpp` |

---

## 14. Simulation (optional)

| Area | Path |
|------|------|
| CARLA / Zenoh / ROS2 bridges | `Simulation/` |

---

## Runtime data flow

```
Camera / video ──► preprocess ──► AutoDrive ║ AutoSteer ║ AutoSpeed (ONNX)
                                      │           │            │
                                      └──── fusion (lon/lat) ──┘
                                                │
                                         planning (IDM + MPC)
                                                │
                         warnings: FCW/AEB/LLDW/RLDW + steer/accel CMD
                                                │
                              FileInterface / CanInterface (TCP :59000)
                                                │
                         Virtual ECU / Arduino / ESP32 / Zephyr gateway
```

---

## Scope note

This tree is the **open-source, demo-capable L2 stack** (mapless, monocular, hybrid E2E).  
MCU trees are **vehicle gateways** (not neural inference). QNX/RPi docs are **host port scaffolds**.  
Series-production ISO 26262 / ISO 8800 certification artifacts are on the upstream roadmap, not fully in-repo.
