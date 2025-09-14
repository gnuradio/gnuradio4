#ifndef GNURADIO_DIGITAL_CRC_HPP
#define GNURADIO_DIGITAL_CRC_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace gr::digital {

struct CrcState {
    unsigned     num_bits         = 0;
    std::uint64_t poly            = 0;
    std::uint64_t initial_value   = 0;
    std::uint64_t final_xor       = 0;
    bool          input_reflected = false;
    bool          result_reflected= false;
};

struct Crc {
    CrcState                           st{};
    std::array<std::uint64_t, 256>     table{};
    std::uint64_t                      mask = 0;
    std::uint64_t                      reg  = 0;

    void start() {
        if (st.num_bits == 0 || st.num_bits > 64) throw std::invalid_argument("crc width");
        if (st.num_bits < 8)                       throw std::invalid_argument("crc width < 8");
        mask = (st.num_bits == 64) ? ~0ull : ((1ull << st.num_bits) - 1ull);
        reg  = st.initial_value & mask;

        if (st.input_reflected) {
            const auto poly_r = reflect(st.poly & mask, st.num_bits);
            for (std::size_t i = 0; i < 256; ++i) {
                std::uint64_t r = static_cast<std::uint64_t>(i);
                for (int b = 0; b < 8; ++b) {
                    r = (r & 1ull) ? ((r >> 1) ^ poly_r) : (r >> 1);
                }
                table[i] = r & mask;
            }
        } else {
            const auto poly = st.poly & mask;
            const std::uint64_t topbit = 1ull << (st.num_bits - 1);
            for (std::size_t i = 0; i < 256; ++i) {
                std::uint64_t r = static_cast<std::uint64_t>(i) << (st.num_bits - 8);
                for (int b = 0; b < 8; ++b) {
                    r = (r & topbit) ? ((r << 1) ^ poly) : (r << 1);
                    r &= mask;
                }
                table[i] = r & mask;
            }
        }
    }

    void stop() {}

    std::uint64_t processOne(std::uint8_t byte) noexcept {
        if (st.input_reflected) {
            const std::uint64_t idx = (reg ^ byte) & 0xFFull;
            reg = (reg >> 8) ^ table[idx];
        } else {
            const std::uint64_t idx =
                ((reg >> (st.num_bits - 8)) ^ static_cast<std::uint64_t>(byte)) & 0xFFull;
            reg = ((reg << 8) & mask) ^ table[idx];
        }
        return reg;
    }

    std::uint64_t compute(const std::uint8_t* data, std::size_t len) noexcept {
        reg = st.initial_value & mask;
        for (std::size_t i = 0; i < len; ++i) processOne(data[i]);
        std::uint64_t out = reg & mask;
        if (st.input_reflected != st.result_reflected) out = reflect(out, st.num_bits);
        out ^= st.final_xor;
        return out & mask;
    }

    static std::uint64_t reflect(std::uint64_t x, unsigned width) noexcept {
        std::uint64_t r = 0;
        for (unsigned i = 0; i < width; ++i) { r = (r << 1) | (x & 1ull); x >>= 1; }
        return r;
    }
};

} // namespace gr::digital

#endif
