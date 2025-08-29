#ifndef GNURADIO_DIGITAL_BINARYSLICER_HPP
#define GNURADIO_DIGITAL_BINARYSLICER_HPP

#include <cstdint>

namespace gr::digital {

// Binary slicer: returns 0 if (x < threshold), else 1.
struct BinarySlicer {
    float threshold = 0.0f;

    void start(float th = 0.0f) { threshold = th; }
    void stop() {}

    std::uint8_t processOne(float x) const noexcept {
        return (x >= threshold) ? 1u : 0u;
    }
};

} // namespace gr::digital

#endif // GNURADIO_DIGITAL_BINARYSLICER_HPP



