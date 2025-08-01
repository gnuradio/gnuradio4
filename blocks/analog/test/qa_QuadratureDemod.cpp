#include <boost/ut.hpp>
#include <cmath>
#include <complex>
#include <numbers>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/analog/FrequencyMod.hpp>
#include <gnuradio-4.0/analog/QuadratureDemod.hpp>

using namespace gr;
using namespace gr::testing;
using namespace gr::blocks::analog;
using namespace boost::ut;

template<typename BlockT, typename InVecT, typename OutT>
std::vector<OutT> run_vector(const InVecT& drive, property_map cfg = {})
{
    Graph g;

    auto& src = g.emplaceBlock<TagSource<typename InVecT::value_type>>(
        { { "values", drive }, { "n_samples_max", drive.size() } });
    auto& blk  = g.emplaceBlock<BlockT>(std::move(cfg));
    auto& sink =
        g.emplaceBlock<TagSink<OutT, ProcessFunction::USE_PROCESS_BULK>>();

    [[maybe_unused]] auto c1 =
        g.connect<"out">(src).template to<"in">(blk);
    [[maybe_unused]] auto c2 =
        g.connect<"out">(blk).template to<"in">(sink);

    scheduler::Simple sch{ std::move(g) };
    expect(bool{ sch.runAndWait() });

    return sink._samples;
}

static constexpr auto fnear = [](float a, float b, float tol = 1e-5f) {
    return std::fabs(a - b) <= tol;
};

suite quad_demod = [] {

    "frequency-mod ↔ quadrature-demod"_test = [] {
        constexpr float  fs   = 8'000.0f;                               // Hz
        constexpr float  f    = 1'000.0f;                               // Hz
        constexpr float  sens = std::numbers::pi_v<float> / 4.0f;
        constexpr float  gain = 1.0f / sens;       // makes demod output unity
        constexpr size_t N    = 200;

        std::vector<float> drive;
        drive.reserve(N);
        for (size_t n = 0; n < N; ++n)
            drive.push_back(
                std::cos(2.0f * std::numbers::pi_v<float> * f *
                         (static_cast<float>(n) / fs)));

        Graph g;
        auto& src = g.emplaceBlock<TagSource<float>>(
            { { "values", drive }, { "n_samples_max", N } });
        auto& fm  = g.emplaceBlock<FrequencyMod<float>>(
            { { "sensitivity", sens } });
        auto& qd  = g.emplaceBlock<QuadratureDemod>(              // ← alias removed
            { { "gain", gain } });
        auto& sink =
            g.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_BULK>>();

        [[maybe_unused]] auto c1 =
            g.connect<"out">(src).template to<"in">(fm);
        [[maybe_unused]] auto c2 =
            g.connect<"out">(fm ).template to<"in">(qd);
        [[maybe_unused]] auto c3 =
            g.connect<"out">(qd ).template to<"in">(sink);

        scheduler::Simple sch{ std::move(g) };
        expect(bool{ sch.runAndWait() });

        std::vector<float> ref = drive;
        if (!ref.empty())
            ref.front() = 0.0f;

        const auto& y = sink._samples;

        expect(y.size() == N);

        for (size_t i = 0; i < N; ++i)
            expect(fnear(y[i], ref[i]))
                << "k=" << i << " got " << y[i] << " ref " << ref[i];
    };
};

int main() { }   // Boost.UT auto-runs suites
