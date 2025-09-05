#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/measure/MeasEvm.hpp>
#include <gnuradio-4.0/digital/core/Constellation.hpp>

#include <vector>
#include <complex>
#include <random>
#include <cmath>
#include <cstddef>

using namespace boost::ut;
using gr::digital::MeasEvmCC;
using gr::digital::EvmMode;
using gr::digital::QPSK_Gray;
using gr::digital::QAM16_Gray;
using gr::digital::cfloat;

template <class C>
static std::vector<cfloat> pts_from(const C& c) {
    return std::vector<cfloat>(c.points.begin(), c.points.end());
}

static std::vector<cfloat> rand_symbols_from(const std::vector<cfloat>& pts, std::size_t n, unsigned seed=42)
{
    std::mt19937 rng(seed);
    std::uniform_int_distribution<std::size_t> pick(0, pts.size()-1);
    std::vector<cfloat> out; out.reserve(n);
    for (std::size_t i=0;i<n;++i) out.push_back(pts[pick(rng)]);
    return out;
}

static std::vector<cfloat> add_awgn(const std::vector<cfloat>& x, float sigma, unsigned seed=99)
{
    std::mt19937 rng(seed);
    std::normal_distribution<float> N(0.0f, sigma);
    std::vector<cfloat> y; y.reserve(x.size());
    for (const auto& s : x) y.emplace_back(s.real()+N(rng), s.imag()+N(rng));
    return y;
}

const suite MeasEvmSuite = [] {
    "QPSK: zero EVM for ideal points"_test = [] {
        auto c   = QPSK_Gray();
        auto pts = pts_from(c);
        auto syms= rand_symbols_from(pts, 1000, 7);

        MeasEvmCC evm; evm.start(pts, EvmMode::Percent);
        bool all_zero = true;
        for (const auto& s : syms) {
            const float e = evm.processOne(s);
            if (e != 0.0f) { all_zero = false; break; }
        }
        expect(all_zero);
    };

    "QPSK: non-zero EVM when scaled/rotated"_test = [] {
        auto c   = QPSK_Gray();
        auto pts = pts_from(c);
        auto syms= rand_symbols_from(pts, 1000, 11);

        const cfloat g{3.0f, 2.0f};
        MeasEvmCC evm; evm.start(pts, EvmMode::Percent);
        std::size_t nz = 0;
        for (const auto& s : syms) {
            const float e = evm.processOne(s * g);
            if (e > 0.0f) ++nz;
        }
        expect(nz == syms.size());
    };

    "QPSK: AWGN -> 0 < EVM% < 50"_test = [] {
        auto c   = QPSK_Gray();
        auto pts = pts_from(c);
        auto syms= rand_symbols_from(pts, 1000, 13);
        auto y   = add_awgn(syms, /*sigma*/0.10f, 21);

        MeasEvmCC evm; evm.start(pts, EvmMode::Percent);
        bool ok = true;
        for (const auto& s : y) {
            const float e = evm.processOne(s);
            if (!(e > 0.0f && e < 50.0f)) { ok = false; break; }
        }
        expect(ok);
    };

    "QAM16: AWGN -> 0 < EVM% < 50"_test = [] {
        auto c   = QAM16_Gray();
        auto pts = pts_from(c);
        auto syms= rand_symbols_from(pts, 1000, 17);
        auto y   = add_awgn(syms, 0.10f, 23);

        MeasEvmCC evm; evm.start(pts, EvmMode::Percent);
        bool ok = true;
        for (const auto& s : y) {
            const float e = evm.processOne(s);
            if (!(e > 0.0f && e < 50.0f)) { ok = false; break; }
        }
        expect(ok);
    };
};
