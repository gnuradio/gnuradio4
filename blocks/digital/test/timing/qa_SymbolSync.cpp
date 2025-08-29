#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/timing/SymbolSync.hpp>

#include <vector>
#include <complex>
#include <cmath>
#include <cstddef>

using namespace boost::ut;

namespace {

static std::vector<float> make_nrz_ff(std::size_t n, int sps = 2)
{
    std::vector<float> v;
    v.reserve(n);
    float cur = +1.0f;
    int run = 0;
    for (std::size_t i = 0; i < n; ++i) {
        v.push_back(cur);
        if (++run == sps) {
            cur = -cur;
            run = 0;
        }
    }
    return v;
}

static std::vector<std::complex<float>> make_nrz_cc(std::size_t n, int sps = 2)
{
    std::vector<std::complex<float>> v;
    v.reserve(n);
    std::complex<float> cur{+1.0f, +1.0f};
    int run = 0;
    for (std::size_t i = 0; i < n; ++i) {
        v.push_back(cur);
        if (++run == sps) {
            cur = -cur;
            run = 0;
        }
    }
    return v;
}

} // namespace

const suite SymbolSyncSuite = [] {
    "SymbolSyncf: sps=2 emits ~N/2 and follows NRZ pattern"_test = [] {
        gr::digital::SymbolSyncf ss;
        ss.start(2.0f, 0.0f); // loop params disabled

        const auto in = make_nrz_ff(4000, 2);
        std::vector<float> out;
        out.reserve(in.size());

        float y{};
        for (const auto& x : in) {
            if (ss.processOne(x, y)) out.push_back(y);
        }

        expect(out.size() >= 1998u && out.size() <= 2001u) << "output count";

        std::size_t ok = 0;
        for (std::size_t k = 0; k < out.size(); ++k) {
            const float exp = (k % 2 == 0) ? +1.0f : -1.0f;
            if (std::fabs(out[k] - exp) < 1e-6f) ++ok;
        }
        expect(ok >= (out.size() * 9) / 10) << "pattern match >= 90%";
        ss.stop();
    };

    "SymbolSynccf: sps=2 emits ~N/2 and follows complex NRZ pattern"_test = [] {
        gr::digital::SymbolSynccf ss;
        ss.start(2.0f, 0.0f); // loop params disabled

        const auto in = make_nrz_cc(4000, 2);
        std::vector<std::complex<float>> out;
        out.reserve(in.size());

        std::complex<float> y{};
        for (const auto& x : in) {
            if (ss.processOne(x, y)) out.push_back(y);
        }

        expect(out.size() >= 1998u && out.size() <= 2001u) << "output count";

        std::size_t ok = 0;
        for (std::size_t k = 0; k < out.size(); ++k) {
            const auto exp =
                (k % 2 == 0) ? std::complex<float>{+1.0f, +1.0f}
                             : std::complex<float>{-1.0f, -1.0f};
            if (std::fabs(out[k].real() - exp.real()) < 1e-6f &&
                std::fabs(out[k].imag() - exp.imag()) < 1e-6f)
                ++ok;
        }
        expect(ok >= (out.size() * 9) / 10) << "pattern match >= 90%";
        ss.stop();
    };
};
