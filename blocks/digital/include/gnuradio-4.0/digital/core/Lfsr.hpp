#ifndef GNURADIO_DIGITAL_LFSR_HPP
#define GNURADIO_DIGITAL_LFSR_HPP

#include <cstdint>

namespace gr::digital {

namespace lfsr_type { enum class Type : int { Fibonacci, Galois }; }

template<lfsr_type::Type V> struct LfsrState;
template<lfsr_type::Type V> struct LfsrGen;
template<lfsr_type::Type V> struct LfsrScrambler;
template<lfsr_type::Type V> struct LfsrDescrambler;

namespace detail {
static inline constexpr std::uint8_t parity64(std::uint64_t v) noexcept {
    v ^= v >> 32; v ^= v >> 16; v ^= v >> 8; v ^= v >> 4; v &= 0xFu;
    return static_cast<std::uint8_t>((0x6996u >> v) & 1u);
}
}

template<lfsr_type::Type V>
struct LfsrState {
    std::uint64_t mask = 0;
    std::uint64_t seed = 1;
    std::uint8_t  len  = 0;
    std::uint64_t sr   = 0;

    void start() noexcept { sr = seed; }
    void stop()  noexcept {}
    void advance(std::size_t n) noexcept { while (n--) (void)step_(); }
    std::uint64_t state() const noexcept { return sr; }

private:
    std::uint8_t step_() noexcept {
        if constexpr (V == lfsr_type::Type::Fibonacci) {
            const std::uint8_t out = static_cast<std::uint8_t>(sr & 1u);
            const std::uint8_t nb  = detail::parity64(sr & mask);
            sr = (sr >> 1) | (static_cast<std::uint64_t>(nb) << len);
            return out;
        } else {
            const std::uint8_t out = static_cast<std::uint8_t>(sr & 1u);
            sr >>= 1;
            if (out) sr ^= mask;
            return out;
        }
    }

    template<lfsr_type::Type> friend struct LfsrGen;
    template<lfsr_type::Type> friend struct LfsrScrambler;
    template<lfsr_type::Type> friend struct LfsrDescrambler;
};

template<lfsr_type::Type V>
struct LfsrGen {
    LfsrState<V> st;
    void start() noexcept { st.start(); }
    void stop()  noexcept { st.stop();  }
    std::uint8_t processOne() noexcept { return st.step_(); }
    std::uint64_t state() const noexcept { return st.state(); }
};

template<lfsr_type::Type V>
struct LfsrScrambler {
    LfsrState<V> st;
    void start() noexcept { st.start(); }
    void stop()  noexcept { st.stop();  }
    std::uint8_t processOne(std::uint8_t in) noexcept {
        if constexpr (V == lfsr_type::Type::Fibonacci) {
            const std::uint8_t p = detail::parity64(st.sr & st.mask);
            const std::uint8_t y = static_cast<std::uint8_t>(p ^ (in & 1u));
            st.sr = (st.sr >> 1) | (static_cast<std::uint64_t>(y) << st.len);
            return y;
        } else {
            const std::uint8_t s0 = static_cast<std::uint8_t>(st.sr & 1u);
            const std::uint8_t y  = static_cast<std::uint8_t>(s0 ^ (in & 1u));
            st.sr >>= 1;
            if (y) st.sr ^= st.mask;
            return y;
        }
    }
    std::uint64_t state() const noexcept { return st.state(); }
};

template<lfsr_type::Type V>
struct LfsrDescrambler {
    LfsrState<V> st;
    void start() noexcept { st.start(); }
    void stop()  noexcept { st.stop();  }
    std::uint8_t processOne(std::uint8_t in) noexcept {
        if constexpr (V == lfsr_type::Type::Fibonacci) {
            const std::uint8_t p = detail::parity64(st.sr & st.mask);
            const std::uint8_t x = static_cast<std::uint8_t>(p ^ (in & 1u));
            st.sr = (st.sr >> 1) | (static_cast<std::uint64_t>(in & 1u) << st.len);
            return x;
        } else {
            const std::uint8_t s0 = static_cast<std::uint8_t>(st.sr & 1u);
            const std::uint8_t x  = static_cast<std::uint8_t>(s0 ^ (in & 1u));
            st.sr >>= 1;
            if (in & 1u) st.sr ^= st.mask;
            return x;
        }
    }
    std::uint64_t state() const noexcept { return st.state(); }
};

using LfsrGenF = LfsrGen<lfsr_type::Type::Fibonacci>;
using LfsrGenG = LfsrGen<lfsr_type::Type::Galois>;
using LfsrScramblerF = LfsrScrambler<lfsr_type::Type::Fibonacci>;
using LfsrScramblerG = LfsrScrambler<lfsr_type::Type::Galois>;
using LfsrDescramblerF = LfsrDescrambler<lfsr_type::Type::Fibonacci>;
using LfsrDescramblerG = LfsrDescrambler<lfsr_type::Type::Galois>;

namespace primitive_polynomials { inline constexpr std::uint64_t poly_5 = 0x29; }

} // namespace gr::digital

#endif
