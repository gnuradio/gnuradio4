#ifndef GNURADIO_DIGITAL_DIFFCODING_HPP
#define GNURADIO_DIGITAL_DIFFCODING_HPP

#include <cstdint>
#include <stdexcept>

namespace gr::digital {

struct DiffEncoder {
    unsigned modulus = 2;
    std::uint32_t prev = 0;

    void start(unsigned m, std::uint32_t seed = 0) {
        if (m < 2) throw std::invalid_argument("DiffEncoder: modulus must be >= 2");
        modulus = m;
        prev = seed % modulus;
    }

    void stop() {}

    std::uint32_t processOne(std::uint32_t in) noexcept {
        const auto xin = in % modulus;
        const auto out = (xin + prev) % modulus;
        prev = out;
        return out;
    }
};

struct DiffDecoder {
    unsigned modulus = 2;
    std::uint32_t prev = 0;

    void start(unsigned m, std::uint32_t seed = 0) {
        if (m < 2) throw std::invalid_argument("DiffDecoder: modulus must be >= 2");
        modulus = m;
        prev = seed % modulus;
    }

    void stop() {}

    std::uint32_t processOne(std::uint32_t in) noexcept {
        const auto yin = in % modulus;
        const auto out = (yin + modulus - prev) % modulus;
        prev = yin;
        return out;
    }
};

} // namespace gr::digital

#endif // GNURADIO_DIGITAL_DIFFCODING_HPP
