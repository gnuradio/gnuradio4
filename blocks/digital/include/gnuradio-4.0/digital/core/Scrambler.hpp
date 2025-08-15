#ifndef GNURADIO_DIGITAL_SCRAMBLER_HPP
#define GNURADIO_DIGITAL_SCRAMBLER_HPP

#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <complex>
#include <stdexcept>

namespace gr::digital {

namespace detail {
inline std::uint8_t parity64(std::uint64_t v) noexcept {
    v ^= v >> 32; v ^= v >> 16; v ^= v >> 8; v ^= v >> 4; v &= 0xFu;
    return static_cast<std::uint8_t>((0x6996u >> v) & 1u);
}
} // namespace detail

// -----------------------------------------------------------------------------
// Additive scrambler (bytes & soft symbols) - unchanged (your version was OK)
// -----------------------------------------------------------------------------
struct AdditiveScramblerState {
    std::uint64_t mask = 0;
    std::uint64_t seed = 1;
    std::uint8_t  len  = 0;
    std::int64_t  count = 0;
    std::uint8_t  bits_per_byte = 1;
};

struct AdditiveScramblerBB {
    AdditiveScramblerState st{};
    std::uint64_t sr = 0;
    std::uint64_t reg_mask = 0;
    std::int64_t processed = 0;

    void start() {
        if (st.len > 63) throw std::invalid_argument("len");
        if (st.bits_per_byte == 0 || st.bits_per_byte > 8) throw std::invalid_argument("bpb");
        const unsigned width = static_cast<unsigned>(st.len + 1);
        reg_mask = (width == 64) ? ~0ull : ((1ull << width) - 1ull);
        sr = st.seed & (reg_mask >> 1);
        processed = 0;
    }
    void stop() {}

    inline std::uint8_t next_lfsr_bit() noexcept {
        const auto out = static_cast<std::uint8_t>(sr & 1u);
        const auto nb  = detail::parity64(sr & st.mask);
        sr = ((sr >> 1) | (static_cast<std::uint64_t>(nb) << st.len)) & reg_mask;
        return out;
    }

    std::uint8_t processOne(std::uint8_t in) noexcept {
        std::uint8_t w = 0;
        for (std::uint8_t i = 0; i < st.bits_per_byte; ++i)
            w ^= static_cast<std::uint8_t>(next_lfsr_bit() << i);
        const auto out = static_cast<std::uint8_t>(in ^ w);
        if (st.count > 0 && ++processed >= st.count) {
            sr = st.seed & (reg_mask >> 1);
            processed = 0;
        }
        return out;
    }

    void process(const std::uint8_t* in, std::uint8_t* out, std::size_t n) noexcept {
        for (std::size_t i = 0; i < n; ++i) out[i] = processOne(in[i]);
    }
};

template <class T>
struct AdditiveScramblerT {
    static_assert(!std::is_same_v<T, std::uint8_t>, "use AdditiveScramblerBB for bytes");
    AdditiveScramblerState st{};
    std::uint64_t sr = 0;
    std::uint64_t reg_mask = 0;
    std::int64_t processed = 0;

    void start() {
        if (st.len > 63) throw std::invalid_argument("len");
        const unsigned width = static_cast<unsigned>(st.len + 1);
        reg_mask = (width == 64) ? ~0ull : ((1ull << width) - 1ull);
        sr = st.seed & (reg_mask >> 1);
        processed = 0;
    }
    void stop() {}

    inline std::uint8_t next_lfsr_bit() noexcept {
        const auto out = static_cast<std::uint8_t>(sr & 1u);
        const auto nb  = detail::parity64(sr & st.mask);
        sr = ((sr >> 1) | (static_cast<std::uint64_t>(nb) << st.len)) & reg_mask;
        return out;
    }

    static inline T flip(const T& x) noexcept { return static_cast<T>(-x); }
    static inline std::complex<float>  flip(const std::complex<float>&  x) noexcept { return -x; }
    static inline std::complex<double> flip(const std::complex<double>& x) noexcept { return -x; }

    T processOne(const T& in) noexcept {
        const auto bit = next_lfsr_bit();
        const auto out = bit ? flip(in) : in;
        if (st.count > 0 && ++processed >= st.count) {
            sr = st.seed & (reg_mask >> 1);
            processed = 0;
        }
        return out;
    }

    void process(const T* in, T* out, std::size_t n) noexcept {
        for (std::size_t i = 0; i < n; ++i) out[i] = processOne(in[i]);
    }
};

using AdditiveScramblerFF = AdditiveScramblerT<float>;
using AdditiveScramblerII = AdditiveScramblerT<std::int32_t>;
using AdditiveScramblerSS = AdditiveScramblerT<std::int16_t>;
using AdditiveScramblerCC = AdditiveScramblerT<std::complex<float>>;
template <class T>
using AdditiveScrambler =
    std::conditional_t<std::is_same_v<T, std::uint8_t>, AdditiveScramblerBB, AdditiveScramblerT<T>>;


// -----------------------------------------------------------------------------
// Self-synchronizing scrambler/descrambler (bitwise)
// Right-shift, insert new bit at MSB, taps taken from (reg >> 1)
// This matches GR-3.x semantics (e.g., CCSDS-7: mask=0x8A, seed=0x7F, len=7).
// -----------------------------------------------------------------------------
struct ScramblerBBState {
    std::uint64_t mask = 0; // taps over previous scrambled bits
    std::uint64_t seed = 0; // initial shift register contents
    std::uint8_t  len  = 0; // order (highest tap distance minus 1)
};

struct ScramblerBB {
    ScramblerBBState st{};
    std::uint64_t reg = 0;
    std::uint64_t reg_mask = 0;

    void start() {
        if (st.len == 0 || st.len > 63) throw std::invalid_argument("len");
        const unsigned width = static_cast<unsigned>(st.len + 1);
        reg_mask = (width == 64) ? ~0ull : ((1ull << width) - 1ull);
        // keep only the lower 'len' bits of the seed
        reg = st.seed & (reg_mask >> 1);
    }
    void stop() {}

    // y[n] = x[n] XOR parity((reg >> 1) & mask)
    // reg holds previous scrambled bits; new y goes into MSB (bit 'len')
    std::uint8_t processOne(std::uint8_t in) noexcept {
        const auto p = detail::parity64((reg >> 1) & st.mask);
        const auto y = static_cast<std::uint8_t>((in & 1u) ^ p);
        reg = ((reg >> 1) | (static_cast<std::uint64_t>(y) << st.len)) & reg_mask;
        return y;
    }

    void process(const std::uint8_t* in, std::uint8_t* out, std::size_t n) noexcept {
        for (std::size_t i = 0; i < n; ++i) out[i] = processOne(in[i]);
    }
};

struct DescramblerBB {
    ScramblerBBState st{};
    std::uint64_t reg = 0;
    std::uint64_t reg_mask = 0;

    void start() {
        if (st.len == 0 || st.len > 63) throw std::invalid_argument("len");
        const unsigned width = static_cast<unsigned>(st.len + 1);
        reg_mask = (width == 64) ? ~0ull : ((1ull << width) - 1ull);
        reg = st.seed & (reg_mask >> 1);
    }
    void stop() {}

    // x[n] = y[n] XOR parity((reg >> 1) & mask)
    // then update with received y[n] at MSB
    std::uint8_t processOne(std::uint8_t s) noexcept {
        const auto p = detail::parity64((reg >> 1) & st.mask);
        const auto x = static_cast<std::uint8_t>((s & 1u) ^ p);
        reg = ((reg >> 1) | (static_cast<std::uint64_t>(s & 1u) << st.len)) & reg_mask;
        return x;
    }

    void process(const std::uint8_t* in, std::uint8_t* out, std::size_t n) noexcept {
        for (std::size_t i = 0; i < n; ++i) out[i] = processOne(in[i]);
    }
};

} // namespace gr::digital

#endif // GNURADIO_DIGITAL_SCRAMBLER_HPP
