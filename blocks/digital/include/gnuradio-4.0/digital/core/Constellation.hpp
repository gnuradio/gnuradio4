#ifndef GNURADIO_DIGITAL_CONSTELLATION_HPP
#define GNURADIO_DIGITAL_CONSTELLATION_HPP

#include <array>
#include <complex>
#include <cstdint>
#include <algorithm>
#include <numeric>
#include <cmath>

namespace gr::digital {

using cfloat = std::complex<float>;

enum class Normalization {
    None,
    Power,      // mean(|x|^2) == 1
    Amplitude   // mean(|x|)   == 1
};

// POD descriptor: N points + labels (bit patterns / symbol ids)
template <std::size_t N>
struct Constellation {
    std::array<cfloat, N> points{};
    std::array<std::uint32_t, N> labels{}; // index -> label (e.g., Gray code)

    // Map (index -> point/label)
    constexpr cfloat point(std::size_t i) const { return points[i]; }
    constexpr std::uint32_t label(std::size_t i) const { return labels[i]; }

    // Optional helpers
    std::size_t index_of_label(std::uint32_t lab) const {
        for (std::size_t i = 0; i < N; ++i) if (labels[i] == lab) return i;
        return N; // not found
    }

    // Derived info
    float avg_power() const {
        float s = 0.f;
        for (auto z : points) s += std::norm(z);
        return s / float(N);
    }

    float avg_amplitude() const {
        float s = 0.f;
        for (auto z : points) s += std::abs(z);
        return s / float(N);
    }

    // Return a copy with requested normalization
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

// Stateless closest-Euclidean slicer
template <std::size_t N>
inline std::size_t closest_euclidean_index(const Constellation<N>& C,
                                           cfloat sample) {
    std::size_t best = 0;
    float bestd = std::numeric_limits<float>::infinity();
    for (std::size_t i = 0; i < N; ++i) {
        const float d = std::norm(sample - C.points[i]);
        if (d < bestd) { bestd = d; best = i; }
    }
    return best;
}

template <std::size_t N>
inline std::uint32_t slice_label_euclidean(const Constellation<N>& C,
                                           cfloat sample) {
    return C.labels[closest_euclidean_index(C, sample)];
}

// ---- Canned constellations (raw coordinates; scale via .normalized(...)) ----

// BPSK: [-1, +1] with labels [0, 1]
inline Constellation<2> BPSK() {
    Constellation<2> c;
    c.points = { cfloat{-1.f, 0.f}, cfloat{+1.f, 0.f} };
    c.labels = { 0u, 1u };
    return c;
}

// Gray QPSK (matches python psk_4_0 mapping)
// Points: [-1-1j, 1-1j, -1+1j, 1+1j]; labels: [0,1,2,3]
inline Constellation<4> QPSK_Gray() {
    Constellation<4> c;
    c.points = {
        cfloat{-1.f,-1.f}, cfloat{+1.f,-1.f},
        cfloat{-1.f,+1.f}, cfloat{+1.f,+1.f}
    };
    c.labels = { 0u, 1u, 2u, 3u };
    return c;
}

// Gray 16-QAM (matches python qam_16_0 mapping)
// Grid: I,Q in {-3,-1,+1,+3}; labels per qam_constellations.py::qam_16_0x0_0_1_2_3
inline Constellation<16> QAM16_Gray() {
    Constellation<16> c;
    c.points = {
        cfloat{-3,-3}, cfloat{-1,-3}, cfloat{+1,-3}, cfloat{+3,-3},
        cfloat{-3,-1}, cfloat{-1,-1}, cfloat{+1,-1}, cfloat{+3,-1},
        cfloat{-3,+1}, cfloat{-1,+1}, cfloat{+1,+1}, cfloat{+3,+1},
        cfloat{-3,+3}, cfloat{-1,+3}, cfloat{+1,+3}, cfloat{+3,+3}
    };
    c.labels = {
        0x0u, 0x4u, 0xCu, 0x8u,
        0x1u, 0x5u, 0xDu, 0x9u,
        0x3u, 0x7u, 0xFu, 0xBu,
        0x2u, 0x6u, 0xEu, 0xAu
    };
    return c;
}

} // namespace gr::digital

#endif // GNURADIO_DIGITAL_CONSTELLATION_HPP
