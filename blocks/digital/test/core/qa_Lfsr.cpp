#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/core/Lfsr.hpp>
#include <vector>

using namespace boost::ut;
using namespace gr::digital;

const suite LfsrTestSuite = [] {
    "LFSR Basic Construction"_test = [] {
        "Fibonacci LFSR construction"_test = [] {
            LfsrFibonacci f(0x8E, 0x01, 8);
            expect(f.mask() == 0x8Eu);
            expect(f.seed() == 0x01u);
            expect(f.state() == 0x01u);
            expect(f.length() == 8u);
        };
        "Galois LFSR construction"_test = [] {
            LfsrGalois g(0x8E, 0x01, 8);
            expect(g.mask() == 0x8Eu);
            expect(g.seed() == 0x01u);
            expect(g.state() == 0x01u);
            expect(g.length() == 8u);
        };
        "Invalid register length"_test = [] {
            expect(throws([] { LfsrFibonacci f(0x8E, 0x01, 64); }));
        };
        "Zero seed"_test = [] {
            expect(nothrow([] { LfsrFibonacci f(0x8E, 0x00, 8); }));
        };
    };

    "LFSR Sequence Generation"_test = [] {
        "Fibonacci nonzero progression"_test = [] {
            LfsrFibonacci f(0x19, 0x1, 3); // degree 4 ⇒ reg_len=3
            bool stuck = false;
            for (int i = 0; i < 20; ++i) {
                if (f.state() == 0) { stuck = true; break; }
                (void)f.next_bit();
            }
            expect(!stuck);
        };
        "Galois period check (4-bit)"_test = [] {
            LfsrGalois g(0x9, 0x1, 4);
            const auto seed = g.state();
            const std::size_t period = (1u << 4) - 1u;
            for (std::size_t i = 0; i < period; ++i) (void)g.next_bit();
            expect(g.state() == seed);
        };
    };

    "LFSR Reset and Advance"_test = [] {
        "Reset"_test = [] {
            LfsrFibonacci f(0x8E, 0xAB, 8);
            const auto s0 = f.state();
            for (int i = 0; i < 10; ++i) (void)f.next_bit();
            expect(f.state() != s0);
            f.reset();
            expect(f.state() == s0);
        };
        "Advance"_test = [] {
            LfsrFibonacci a(0x8E, 0x1, 8), b(0x8E, 0x1, 8);
            for (int i = 0; i < 5; ++i) (void)a.next_bit();
            b.advance(5);
            expect(a.state() == b.state());
        };
    };

    "LFSR Scrambling"_test = [] {
        "Fibonacci scramble/descramble"_test = [] {
            LfsrFibonacci s(0x8E, 0x1, 8);
            LfsrFibonacci d(0x8E, 0x1, 8);
            std::vector<std::uint8_t> in = {1,0,1,1,0,0,1,0,1};
            std::vector<std::uint8_t> scr, dec;
            for (auto b : in) scr.push_back(s.next_bit_scramble(b));
            for (auto b : scr) dec.push_back(d.next_bit_descramble(b));
            expect(dec == in);
        };
        "Galois scramble/descramble"_test = [] {
            LfsrGalois s(0x9, 0x1, 4);
            LfsrGalois d(0x9, 0x1, 4);
            std::vector<std::uint8_t> in = {1,0,1,0,1};
            std::vector<std::uint8_t> scr, dec;
            for (auto b : in) scr.push_back(s.next_bit_scramble(b));
            for (auto b : scr) dec.push_back(d.next_bit_descramble(b));
            expect(dec == in);
        };
    };

    "Primitive Polynomials"_test = [] {
        "5-bit primitive (0x29) period 31"_test = [] {
            using namespace primitive_polynomials;
            LfsrFibonacci f(poly_5, 0x1, 4); // degree 5 ⇒ reg_len=4
            const auto seed = f.state();
            const std::size_t period = (1u << 5) - 1u;
            for (std::size_t i = 0; i < period; ++i) (void)f.next_bit();
            expect(f.state() == seed);
        };
    };

    "Legacy Compatibility"_test = [] {
        "Type aliases"_test = [] {
            lfsr lf(0x8E, 0x1, 8);
            glfsr lg(0x9, 0x1, 4);
            auto b1 = lf.next_bit();
            auto b2 = lg.next_bit();
            expect((b1 == 0 || b1 == 1));
            expect((b2 == 0 || b2 == 1));
        };
    };

    "Constexpr Functionality"_test = [] {
        "constexpr path"_test = [] {
            constexpr auto fn = []() constexpr {
                LfsrFibonacci f(0x6, 0x1, 3);
                std::uint8_t r = 0;
                for (int i = 0; i < 3; ++i) r ^= f.next_bit();
                return r;
            };
            constexpr auto r = fn();
            expect(r == (r & 1u));
        };
    };
};

int main() {}
