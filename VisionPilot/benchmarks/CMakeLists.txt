cmake_minimum_required(VERSION 3.22.1)

find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_executable(vp_perf_bench vp_perf_bench.cpp)

target_link_libraries(vp_perf_bench
    PRIVATE
        dsp_simd
        ${OpenCV_LIBS}
)

target_compile_features(vp_perf_bench PRIVATE cxx_std_17)
