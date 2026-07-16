#ifndef VISIONPILOT_ALIGNED_BUFFER_HPP
#define VISIONPILOT_ALIGNED_BUFFER_HPP

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>
#include <vector>

namespace visionpilot::dsp {

// 64-byte aligned allocator — good for AVX-512/AVX2/NEON cache lines and DMA.
template <typename T, std::size_t Alignment = 64>
struct AlignedAllocator {
    using value_type = T;

    AlignedAllocator() noexcept = default;
    template <typename U>
    AlignedAllocator(const AlignedAllocator<U, Alignment>&) noexcept {}

    T* allocate(std::size_t n)
    {
        void* p = nullptr;
#if defined(_MSC_VER)
        p = _aligned_malloc(n * sizeof(T), Alignment);
#else
        if (posix_memalign(&p, Alignment, n * sizeof(T)) != 0) p = nullptr;
#endif
        if (!p) throw std::bad_alloc();
        return static_cast<T*>(p);
    }

    void deallocate(T* p, std::size_t) noexcept
    {
#if defined(_MSC_VER)
        _aligned_free(p);
#else
        std::free(p);
#endif
    }

    template <typename U>
    struct rebind { using other = AlignedAllocator<U, Alignment>; };
};

template <typename T, typename U, std::size_t A>
bool operator==(const AlignedAllocator<T, A>&, const AlignedAllocator<U, A>&) { return true; }
template <typename T, typename U, std::size_t A>
bool operator!=(const AlignedAllocator<T, A>&, const AlignedAllocator<U, A>&) { return false; }

template <typename T>
using AlignedVector = std::vector<T, AlignedAllocator<T, 64>>;

// Reusable CHW scratch — avoids per-frame heap traffic (major latency/memory win).
class ChwArena {
public:
    explicit ChwArena(std::size_t chw_elems = 3 * 512 * 1024)
        : buf_(chw_elems)
    {
    }

    float* data() { return buf_.data(); }
    const float* data() const { return buf_.data(); }
    std::size_t size() const { return buf_.size(); }
    std::size_t bytes() const { return buf_.size() * sizeof(float); }

private:
    AlignedVector<float> buf_;
};

}  // namespace visionpilot::dsp

#endif
