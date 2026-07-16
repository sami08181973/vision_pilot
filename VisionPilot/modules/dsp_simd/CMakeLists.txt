cmake_minimum_required(VERSION 3.22.1)

find_package(OpenCV REQUIRED COMPONENTS core imgproc)

option(VP_ENABLE_SIMD "Enable SIMD/DSP CHW conversion" ON)
option(VP_NATIVE_ARCH "Compile dsp_simd with -march=native (host CPU)" ON)

add_library(dsp_simd
    src/chw_convert.cpp
)

target_include_directories(dsp_simd
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dsp_simd
    PUBLIC
        opencv_core
        opencv_imgproc
)

target_compile_features(dsp_simd PUBLIC cxx_std_17)

if(VP_ENABLE_SIMD)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|amd64|i686")
        if(VP_NATIVE_ARCH AND NOT MSVC)
            target_compile_options(dsp_simd PRIVATE -O3 -march=native)
        elseif(NOT MSVC)
            # Portable baseline that still enables SSE2; AVX2 if compiler allows via -mavx2
            include(CheckCXXCompilerFlag)
            check_cxx_compiler_flag("-mavx2" VP_HAS_AVX2)
            if(VP_HAS_AVX2)
                target_compile_options(dsp_simd PRIVATE -O3 -mavx2 -mfma)
            else()
                target_compile_options(dsp_simd PRIVATE -O3 -msse2)
            endif()
        endif()
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        if(NOT MSVC)
            target_compile_options(dsp_simd PRIVATE -O3 -O3)
            # NEON is default on AArch64
        endif()
    endif()
endif()

message(STATUS "dsp_simd backend compile flags applied (VP_ENABLE_SIMD=${VP_ENABLE_SIMD})")
