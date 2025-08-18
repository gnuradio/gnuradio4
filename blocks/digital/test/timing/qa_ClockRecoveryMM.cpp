#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/timing/ClockRecoveryMM.hpp>
#include <complex>
#include <vector>
#include <numeric>
#include <cstddef>
#include <algorithm>

using namespace boost::ut;
using gr::digital::ClockRecoveryMMf;
using gr::digital::ClockRecoveryMMcf;

template <typename T>
static std::size_t run_recovery(auto&& cr, const std::vector<T>& in, std::vector<T>& out)
{
    out.clear();
    T y{};
    for (const auto& x : in) {
        if (cr.processOne(x, y)) out.push_back(y);
    }
    return out.size();
}

const suite ClockRecoveryMMSuite = [] {
    "float: NRZ pattern, emits ~1/omega of inputs"_test = [] {
        ClockRecoveryMMf cr;
        cr.start(/*omega*/2.0f, /*g_omega*/0.01f, /*mu*/0.5f, /*g_mu*/0.01f, /*rel*/0.02f);

        std::vector<float> in;
        in.reserve(4000);
        for (int i = 0; i < 1000; ++i) {
            in.push_back(1.f); in.push_back(1.f); in.push_back(-1.f); in.push_back(-1.f);
        }

        std::vector<float> out;
        const auto n = run_recovery(cr, in, out);

        expect(n > 1500_u && n < 2500_u); // ~2000 if omega=2
        // Check last 200 emitted decisions are ±1
        const std::size_t check = std::min<std::size_t>(200, out.size());
        for (std::size_t i = out.size() - check; i < out.size(); ++i) {
            expect((out[i] == 1.f) || (out[i] == -1.f));
        }
    };

    "float: constant +1 input converges to +1 decisions"_test = [] {
        ClockRecoveryMMf cr;
        cr.start(2.0f, 0.001f, 0.25f, 0.01f, 0.01f);
        std::vector<float> in(800, 1.0f), out;
        run_recovery(cr, in, out);
        const std::size_t pos =
            static_cast<std::size_t>(std::count(out.begin(), out.end(), 1.0f));
        expect(pos * 10 > 9 * out.size());
    };

    "complex: constant (1+j) input converges to quadrant decision"_test = [] {
        ClockRecoveryMMcf cr;
        cr.start(2.0f, 0.001f, 0.25f, 0.01f, 0.01f);
        using c = std::complex<float>;
        std::vector<c> in(1200, c{1.f, 1.f}), out;
        run_recovery(cr, in, out);
        const c exp{1.f, 1.f};
        const std::size_t ok =
            static_cast<std::size_t>(std::count(out.begin(), out.end(), exp));
        expect(ok * 10 > 9 * out.size());
    };
};

int main() {}


