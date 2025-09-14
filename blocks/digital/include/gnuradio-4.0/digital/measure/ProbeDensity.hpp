#ifndef GNURADIO_DIGITAL_MEASURE_PROBEDENSITY_HPP
#define GNURADIO_DIGITAL_MEASURE_PROBEDENSITY_HPP

#include <cstdint>
#include <algorithm>

namespace gr::digital {

struct ProbeDensityB {
    double alpha{0.01};  
    double y{0.0};       

    void start(double a, double init = 0.0) {
        set_alpha(a);
        y = std::clamp(init, 0.0, 1.0);
    }
    void stop() {}

    inline void processOne(std::uint8_t x) noexcept {
        const double xi = (x & 0x01u) ? 1.0 : 0.0;
        y = alpha * y + (1.0 - alpha) * xi;
    }

    inline double density() const noexcept { return y; }

    void set_alpha(double a) {
        alpha = std::clamp(a, 1e-9, 1.0);
    }
};

} // namespace gr::digital

#endif // GNURADIO_DIGITAL_MEASURE_PROBEDENSITY_HPP
