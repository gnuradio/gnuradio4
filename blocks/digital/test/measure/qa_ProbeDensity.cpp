#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/measure/ProbeDensity.hpp>

#include <vector>
#include <cstdint>
#include <cmath>

using namespace boost::ut;
using gr::digital::ProbeDensityB;

static double ewma_run(double alpha, double init, const std::vector<std::uint8_t>& seq)
{
    double y = init;
    for (auto b : seq) {
        const double xi = (b & 0x01u) ? 1.0 : 0.0;
        y = alpha * y + (1.0 - alpha) * xi;
    }
    return y;
}

const suite ProbeDensitySuite = [] {
    "alpha=1 behaves like 'hold previous (no update from x)'"_test = [] {
        ProbeDensityB p; p.start(1.0, 0.0);
        std::vector<std::uint8_t> seq{0,1,0,1};
        for (auto b : seq) p.processOne(b);
        expect(p.density() == 0.0);
    };

    "all ones: near 1 after a few samples (small alpha)"_test = [] {
        const double alpha = 0.01;
        ProbeDensityB p; p.start(alpha, 0.0);
        std::vector<std::uint8_t> seq{1,1,1,1};
        for (auto b : seq) p.processOne(b);

        const double expected = 1.0 - std::pow(alpha, static_cast<double>(seq.size()));
        expect(std::abs(p.density() - expected) < 1e-9);
        expect(p.density() > 0.95); // sanity bound
    };

    "alternating sequence: matches kernel recurrence"_test = [] {
        const double alpha = 0.01;
        const double init  = 0.0;
        std::vector<std::uint8_t> seq{0,1,0,1,0,1,0,1,0,1};

        ProbeDensityB p; p.start(alpha, init);
        for (auto b : seq) p.processOne(b);

        const double expected = ewma_run(alpha, init, seq);
        expect(std::abs(p.density() - expected) < 1e-12);
    };
};
