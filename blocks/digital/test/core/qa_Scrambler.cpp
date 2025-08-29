#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/core/Scrambler.hpp>

#include <algorithm>
#include <complex>
#include <cstdint>
#include <random>
#include <vector>

using namespace boost::ut;
using namespace gr::digital;

namespace {
std::vector<std::uint8_t> rand_bits(std::size_t n, unsigned seed = 123)
{
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> d(0, 1);
    std::vector<std::uint8_t> v(n);
    for (auto& b : v) b = static_cast<std::uint8_t>(d(rng));
    return v;
}

std::vector<std::uint8_t> rand_bytes(std::size_t n, unsigned seed = 321)
{
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> d(0, 255);
    std::vector<std::uint8_t> v(n);
    for (auto& b : v) b = static_cast<std::uint8_t>(d(rng));
    return v;
}

std::vector<float> rand_floats(std::size_t n, unsigned seed = 777)
{
    std::mt19937 rng(seed);
    std::normal_distribution<float> d(0.0f, 1.0f);
    std::vector<float> v(n);
    for (auto& x : v) x = d(rng);
    return v;
}
} // namespace

const suite ScramblerSuite = [] {
    "SelfSync roundtrip (CCSDS 7-bit)"_test = [] {
    const std::size_t N = 1000;
    auto in = rand_bits(N);

    ScramblerBB s; s.st = {0x8a, 0x7fu, 7}; s.start();
    DescramblerBB d; d.st = s.st; d.start();

    std::vector<std::uint8_t> out; out.reserve(N);
    for (auto b : in) {
        auto y = s.processOne(b);
        out.push_back(d.processOne(y));
    }

    // Accept a transient of 0..len+1 (historically 8 for CCSDS-7)
    const std::size_t max_skip = s.st.len + 1; // 8
    auto aligns_with_skip = [&](std::size_t k) {
        if (k >= N) return false;
        for (std::size_t i = k; i < N; ++i)
            if (out[i] != in[i - k]) return false;
        return true;
    };

    std::size_t found = max_skip + 1;
    for (std::size_t k = 0; k <= max_skip; ++k) {
        if (aligns_with_skip(k)) { found = k; break; }
    }

    expect(found <= max_skip) << "roundtrip mismatch (no alignment within 0.." << max_skip << ")";
};


    "Additive byte scrambler roundtrip (bpb=8)"_test = [] {
        const std::size_t N = 1024;
        auto in = rand_bytes(N);

        AdditiveScrambler<std::uint8_t> s; s.st = {0x8a, 0x7fu, 7, 0, 8}; s.start();
        AdditiveScrambler<std::uint8_t> d; d.st = s.st; d.start();

        std::vector<std::uint8_t> out; out.reserve(N);
        for (auto b : in) out.push_back(d.processOne(s.processOne(b)));

        expect(out == in) << "additive byte roundtrip mismatch";
    };

    "Additive soft-symbol scrambler roundtrip (float)"_test = [] {
        const std::size_t N = 1000;
        auto in = rand_floats(N);

        AdditiveScrambler<float> s; s.st = {0x8a, 0x7fu, 7, 0, 1}; s.start();
        AdditiveScrambler<float> d; d.st = s.st; d.start();

        std::vector<float> out; out.reserve(N);
        for (auto x : in) out.push_back(d.processOne(s.processOne(x)));

        bool ok = true;
        for (std::size_t i = 0; i < N; ++i) {
            const float diff = std::abs(out[i] - in[i]);
            if (diff > 1e-6f) { ok = false; break; }
        }
        expect(ok) << "additive float roundtrip mismatch";
    };

    "Additive count reset (bpb=1, repeats every count)"_test = [] {
        const std::size_t N = 200;
        std::vector<std::uint8_t> in(N, 1);

        AdditiveScrambler<std::uint8_t> s; s.st = {0x8a, 0x7fu, 7, 50, 1}; s.start();

        std::vector<std::uint8_t> out; out.reserve(N);
        for (auto b : in) out.push_back(s.processOne(b));

        auto a = std::vector<std::uint8_t>(out.begin(), out.begin() + 50);
        auto b = std::vector<std::uint8_t>(out.begin() + 50, out.begin() + 100);
        auto c = std::vector<std::uint8_t>(out.begin() + 100, out.begin() + 150);
        auto d = std::vector<std::uint8_t>(out.begin() + 150, out.begin() + 200);

        expect(a == b && b == c && c == d) << "pattern not repeating at count boundary";
    };

    "Additive count reset (bpb=3, repeats every count)"_test = [] {
        const std::size_t N = 200;
        std::vector<std::uint8_t> in(N, 5);

        AdditiveScrambler<std::uint8_t> s; s.st = {0x8a, 0x7fu, 7, 50, 3}; s.start();

        std::vector<std::uint8_t> out; out.reserve(N);
        for (auto b : in) out.push_back(s.processOne(b));

        auto a = std::vector<std::uint8_t>(out.begin(), out.begin() + 50);
        auto b = std::vector<std::uint8_t>(out.begin() + 50, out.begin() + 100);
        auto c = std::vector<std::uint8_t>(out.begin() + 100, out.begin() + 150);
        auto d = std::vector<std::uint8_t>(out.begin() + 150, out.begin() + 200);

        expect(a == b && b == c && c == d) << "pattern not repeating at count boundary (bpb=3)";
    };
};
