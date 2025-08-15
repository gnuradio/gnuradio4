#ifndef GNURADIO_DIGITAL_CONSTELLATION_HPP
#define GNURADIO_DIGITAL_CONSTELLATION_HPP

#include <array>
#include <complex>
#include <cstdint>
#include <limits>
#include <cmath>

namespace gr::digital {

using cfloat = std::complex<float>;

enum class Normalization {
    None,
    Power,      
    Amplitude   
};

template <std::size_t N>
struct Constellation {
    std::array<cfloat, N> points{};
    std::array<std::uint32_t, N> labels{}; 

    constexpr cfloat point(std::size_t i) const { return points[i]; }
    constexpr std::uint32_t label(std::size_t i) const { return labels[i]; }

    std::size_t index_of_label(std::uint32_t lab) const {
        for (std::size_t i = 0; i < N; ++i) if (labels[i] == lab) return i;
        return N; // not found
    }

    float avg_power() const {
        float s = 0.f;
        for (auto z : points) s += std::norm(z);
        return s / static_cast<float>(N);
    }

    float avg_amplitude() const {
        float s = 0.f;
        for (auto z : points) s += std::abs(z);
        return s / static_cast<float>(N);
    }

    Constellation<N> normalized(Normalization mode) const {
        if (mode == Normalization::None) return *this;

        Constellation<N> out = *this;
        if (mode == Normalization::Power) {
            const float ap = std::max(1e-30f, avg_power());
            const float g = 1.0f / std::sqrt(ap);
            for (auto& z : out.points) z *= g;
        } else { // Amplitude
            const float aa = std::max(1e-30f, avg_amplitude());
            const float g = 1.0f / aa;
            for (auto& z : out.points) z *= g;
        }
        return out;
    }
};

inline bool finite(cfloat z) noexcept {
    return std::isfinite(z.real()) && std::isfinite(z.imag());
}

struct EuclideanSlicer {
    template <std::size_t N>
    static std::size_t processOneIndex(const Constellation<N>& C, cfloat sample) {
        if (!finite(sample)) return 0; // corner-case fallback

        std::size_t best = 0;
        float bestd = std::numeric_limits<float>::infinity();
        for (std::size_t i = 0; i < N; ++i) {
            const float d = std::norm(sample - C.points[i]);
            if (d < bestd) { bestd = d; best = i; } // stable tie-break
        }
        return best;
    }

    template <std::size_t N>
    static std::uint32_t processOneLabel(const Constellation<N>& C, cfloat sample) {
        return C.labels[processOneIndex(C, sample)];
    }
};

template <std::size_t N>
inline std::size_t closest_euclidean_index(const Constellation<N>& C, cfloat s) {
    return EuclideanSlicer::processOneIndex(C, s);
}
template <std::size_t N>
inline std::uint32_t slice_label_euclidean(const Constellation<N>& C, cfloat s) {
    return EuclideanSlicer::processOneLabel(C, s);
}

constexpr Constellation<2> BPSK() {
    return Constellation<2>{
        /* points */ { cfloat{-1.f, 0.f}, cfloat{+1.f, 0.f} },
        /* labels */ { 0u, 1u }
    };
}

constexpr Constellation<4> QPSK_Gray() {
    return Constellation<4>{
        /* points */ {
            cfloat{-1.f,-1.f}, cfloat{+1.f,-1.f},
            cfloat{-1.f,+1.f}, cfloat{+1.f,+1.f}
        },
        /* labels */ { 0u, 1u, 2u, 3u }
    };
}

constexpr Constellation<16> QAM16_Gray() {
    return Constellation<16>{
        /* points */ {
            cfloat{-3,-3}, cfloat{-1,-3}, cfloat{+1,-3}, cfloat{+3,-3},
            cfloat{-3,-1}, cfloat{-1,-1}, cfloat{+1,-1}, cfloat{+3,-1},
            cfloat{-3,+1}, cfloat{-1,+1}, cfloat{+1,+1}, cfloat{+3,+1},
            cfloat{-3,+3}, cfloat{-1,+3}, cfloat{+1,+3}, cfloat{+3,+3}
        },
        /* labels */ {
            0x0u, 0x4u, 0xCu, 0x8u,
            0x1u, 0x5u, 0xDu, 0x9u,
            0x3u, 0x7u, 0xFu, 0xBu,
            0x2u, 0x6u, 0xEu, 0xAu
        }
    };
}

} // namespace gr::digital

#endif // GNURADIO_DIGITAL_CONSTELLATION_HPP
