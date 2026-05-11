cmake_minimum_required(VERSION 3.22.1)

# ─── ONNX Runtime ─────────────────────────────────────────────────────────────
# Set ONNXRUNTIME_ROOT to your ORT installation, e.g.:
#   cmake -DONNXRUNTIME_ROOT=/opt/onnxruntime ..
# Mirrors the pattern used in the 0.9 release.
if(NOT DEFINED ONNXRUNTIME_ROOT)
    set(ONNXRUNTIME_ROOT "" CACHE PATH "Path to ONNX Runtime installation root")
endif()

if(ONNXRUNTIME_ROOT STREQUAL "")
    message(FATAL_ERROR
        "[models] ONNXRUNTIME_ROOT is not set.\n"
        "  Pass it via: cmake -DONNXRUNTIME_ROOT=<path/to/onnxruntime> ..")
endif()

# ─── Library ──────────────────────────────────────────────────────────────────
add_library(models
    src/onnx_session.cpp
    src/auto_drive.cpp
    src/auto_steer.cpp
    src/auto_speed.cpp
)

target_include_directories(models
    PUBLIC
        include
        ${ONNXRUNTIME_ROOT}/include
)

target_link_libraries(models
    PUBLIC
        ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so
)

target_compile_features(models PUBLIC cxx_std_17)
