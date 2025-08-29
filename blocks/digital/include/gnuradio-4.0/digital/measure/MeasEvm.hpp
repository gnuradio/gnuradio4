#ifndef GNURADIO_DIGITAL_MEASURE_MEASEVM_HPP
#define GNURADIO_DIGITAL_MEASURE_MEASEVM_HPP

#include <vector>
#include <complex>
#include <cmath>
#include <cstddef>
#include <algorithm>

namespace gr::digital {

enum class EvmMode { Percent, dB };

struct MeasEvmCC {
    using cfloat = std::complex<float>;

    EvmMode mode{EvmMode::Percent};
    float   Aref{1.0f};                 
    std::vector<cfloat> ref_;           

    template <class Range>
    void start(const Range& points, EvmMode m = EvmMode::Percent) {
        ref_.assign(std::begin(points), std::end(points));
        mode = m;

        double acc = 0.0;
        for (const auto& s : ref_) acc += static_cast<double>(std::norm(s));
        const double mean_pwr = (ref_.empty() ? 1.0 : acc / static_cast<double>(ref_.size()));
        Aref = static_cast<float>(std::sqrt(std::max(mean_pwr, 1e-30)));
    }

    void stop() { ref_.clear(); Aref = 1.0f; }

    float processOne(const cfloat& y) const noexcept {
        const cfloat* s_near = nearest_(y);
        const float   e_lin  = std::abs(y - *s_near) / Aref;
        if (mode == EvmMode::Percent) return 100.0f * e_lin;
        const float x = std::max(e_lin, 1e-12f);
        return 20.0f * std::log10(x);
    }

private:
    const cfloat* nearest_(const cfloat& y) const noexcept {
        const cfloat* best = &ref_[0];
        float bestd = std::numeric_limits<float>::infinity();
        for (const auto& s : ref_) {
            const float d = std::norm(y - s); 
            if (d < bestd) { bestd = d; best = &s; }
        }
        return best;
    }
};

} // namespace gr::digital

#endif // GNURADIO_DIGITAL_MEASURE_MEASEVM_HPP
