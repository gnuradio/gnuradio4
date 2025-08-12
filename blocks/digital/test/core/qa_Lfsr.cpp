#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/core/Lfsr.hpp>
#include <vector>

using namespace boost::ut;
using namespace gr::digital;

const suite LfsrTestSuite = [] {
    "Construction & lifecycle"_test = [] {
        LfsrGenF gen; gen.st.mask = 0x8E; gen.st.seed = 0x1; gen.st.len = 8; gen.start();
        expect(gen.state() == 0x1u);
        gen.stop();
    };

    "Fibonacci generator progression"_test = [] {
        LfsrGenF gen; gen.st.mask = 0x19; gen.st.seed = 0x1; gen.st.len = 3; gen.start();
        bool stuck = false;
        for (int i = 0; i < 20; ++i) {
            if (gen.state() == 0) { stuck = true; break; }
            (void)gen.processOne();
        }
        expect(!stuck);
    };

    "Galois period (4-bit)"_test = [] {
        LfsrGenG gen; gen.st.mask = 0x9; gen.st.seed = 0x1; gen.st.len = 4; gen.start();
        const auto seed = gen.state();
        const std::size_t period = (1u << 4) - 1u;
        for (std::size_t i = 0; i < period; ++i) (void)gen.processOne();
        expect(gen.state() == seed);
    };

    "Scramble/descramble Fibonacci"_test = [] {
        LfsrScramblerF s; s.st.mask = 0x8E; s.st.seed = 0x1; s.st.len = 8; s.start();
        LfsrDescramblerF d; d.st.mask = 0x8E; d.st.seed = 0x1; d.st.len = 8; d.start();
        std::vector<std::uint8_t> in = {1,0,1,1,0,0,1,0,1}, scr, dec;
        for (auto b : in) scr.push_back(s.processOne(b));
        for (auto b : scr) dec.push_back(d.processOne(b));
        expect(dec == in);
    };

    "Scramble/descramble Galois"_test = [] {
        LfsrScramblerG s; s.st.mask = 0x9; s.st.seed = 0x1; s.st.len = 4; s.start();
        LfsrDescramblerG d; d.st.mask = 0x9; d.st.seed = 0x1; d.st.len = 4; d.start();
        std::vector<std::uint8_t> in = {1,0,1,0,1}, scr, dec;
        for (auto b : in) scr.push_back(s.processOne(b));
        for (auto b : scr) dec.push_back(d.processOne(b));
        expect(dec == in);
    };

    "Primitive poly period (5-bit)"_test = [] {
        LfsrGenF gen; gen.st.mask = primitive_polynomials::poly_5; gen.st.seed = 0x1; gen.st.len = 4; gen.start();
        const auto seed = gen.state();
        const std::size_t period = (1u << 5) - 1u;
        for (std::size_t i = 0; i < period; ++i) (void)gen.processOne();
        expect(gen.state() == seed);
    };
};

int main() {}
