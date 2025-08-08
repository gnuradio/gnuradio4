#ifndef GNURADIO_DIGITAL_LFSR_HPP
#define GNURADIO_DIGITAL_LFSR_HPP

#include <bit>
#include <cstdint>
#include <stdexcept>

namespace gr::digital {

namespace lfsr_type { enum class Type : int { Fibonacci, Galois }; }

template<lfsr_type::Type Variant = lfsr_type::Type::Fibonacci>
class Lfsr {
public:
    using RegisterType = std::uint64_t;

    constexpr Lfsr(RegisterType mask, RegisterType seed, std::uint8_t reg_len)
        : _sr(seed), _mask(mask), _seed(seed), _len(reg_len)
    {
        if (_len > 63) throw std::invalid_argument("reg_len must be <= 63");
    }

    constexpr std::uint8_t next_bit() noexcept {
        if constexpr (Variant == lfsr_type::Type::Fibonacci) return next_bit_fib();
        else return next_bit_gal();
    }

    constexpr std::uint8_t next_bit_scramble(std::uint8_t in) noexcept {
        if constexpr (Variant == lfsr_type::Type::Fibonacci) return next_bit_scramble_fib(in);
        else return next_bit_scramble_gal(in);
    }

    constexpr std::uint8_t next_bit_descramble(std::uint8_t in) noexcept {
        if constexpr (Variant == lfsr_type::Type::Fibonacci) return next_bit_descramble_fib(in);
        else return next_bit_descramble_gal(in);
    }

    constexpr void reset() noexcept { _sr = _seed; }
    constexpr void pre_shift(int n) noexcept { while (n-- > 0) next_bit(); }
    constexpr void advance(std::size_t n) noexcept { while (n-- > 0) next_bit(); }

    constexpr RegisterType mask()  const noexcept { return _mask; }
    constexpr RegisterType seed()  const noexcept { return _seed; }
    constexpr RegisterType state() const noexcept { return _sr; }
    constexpr std::uint8_t length() const noexcept { return _len; }

private:
    RegisterType _sr;
    RegisterType _mask;
    RegisterType _seed;
    std::uint8_t _len;

    static constexpr std::uint8_t parity(RegisterType v) noexcept {
        v ^= v >> 32;
        v ^= v >> 16;
        v ^= v >> 8;
        v ^= v >> 4;
        v &= 0xF;
        return static_cast<std::uint8_t>((0x6996u >> v) & 1u);
    }

    constexpr std::uint8_t next_bit_fib() noexcept {
        const std::uint8_t out = static_cast<std::uint8_t>(_sr & 1u);
        const std::uint8_t nb  = parity(_sr & _mask);
        _sr = (_sr >> 1) | (static_cast<RegisterType>(nb) << _len);
        return out;
    }

    constexpr std::uint8_t next_bit_scramble_fib(std::uint8_t in) noexcept {
        const std::uint8_t p  = parity(_sr & _mask);
        const std::uint8_t y  = static_cast<std::uint8_t>(p ^ (in & 1u));
        _sr = (_sr >> 1) | (static_cast<RegisterType>(y) << _len);
        return y;
    }

    constexpr std::uint8_t next_bit_descramble_fib(std::uint8_t in) noexcept {
        const std::uint8_t p  = parity(_sr & _mask);
        const std::uint8_t x  = static_cast<std::uint8_t>(p ^ (in & 1u));
        _sr = (_sr >> 1) | (static_cast<RegisterType>(in & 1u) << _len);
        return x;
    }

    constexpr std::uint8_t next_bit_gal() noexcept {
        const std::uint8_t out = static_cast<std::uint8_t>(_sr & 1u);
        _sr >>= 1;
        if (out) _sr ^= _mask;
        return out;
    }

    constexpr std::uint8_t next_bit_scramble_gal(std::uint8_t in) noexcept {
        const std::uint8_t s0 = static_cast<std::uint8_t>(_sr & 1u);
        const std::uint8_t y  = static_cast<std::uint8_t>(s0 ^ (in & 1u));
        _sr >>= 1;
        if (y) _sr ^= _mask;
        return y;
    }

    constexpr std::uint8_t next_bit_descramble_gal(std::uint8_t in) noexcept {
        const std::uint8_t s0 = static_cast<std::uint8_t>(_sr & 1u);
        const std::uint8_t x  = static_cast<std::uint8_t>(s0 ^ (in & 1u));
        _sr >>= 1;
        if (in & 1u) _sr ^= _mask;
        return x;
    }
};

using LfsrFibonacci = Lfsr<lfsr_type::Type::Fibonacci>;
using LfsrGalois    = Lfsr<lfsr_type::Type::Galois>;
using lfsr          = LfsrFibonacci;
using glfsr         = LfsrGalois;

namespace primitive_polynomials {
constexpr std::uint64_t poly_5 = 0x29;
}

} // namespace gr::digital

#endif
