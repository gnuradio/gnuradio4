#include <boost/ut.hpp>
#include <cmath>
#include <complex>
#include <numbers>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>
#include <gnuradio-4.0/analog/FrequencyMod.hpp>

using namespace gr;
using namespace gr::testing;
using namespace gr::blocks::analog;
using namespace boost::ut;

template<typename InT , typename OutT>
std::vector<OutT> run_pipeline(const std::vector<InT>& drive,
                               property_map mod_cfg = {})
{
    Graph g;

    auto& src  = g.emplaceBlock<TagSource<InT>>(
        { { "values", drive }, { "n_samples_max", drive.size() } });

    auto& mod  = g.emplaceBlock<FrequencyModF>(std::move(mod_cfg));

    auto& sink =
        g.emplaceBlock<TagSink<OutT, ProcessFunction::USE_PROCESS_BULK>>();

    [[maybe_unused]] auto c1 =
        g.connect<"out">(src).template to<"in">(mod);
    [[maybe_unused]] auto c2 =
        g.connect<"out">(mod).template to<"in">(sink);

    scheduler::Simple sch{ std::move(g) };
    expect(bool{ sch.runAndWait() });

    return sink._samples;
}

suite freqmod_tests = [] {

    "frequency modulator"_test = [] {
        constexpr float k = std::numbers::pi_v<float> / 4.0f;   // sensitivity
        const float src_data[] = { 0.25f, 0.5f, 0.25f,
                                   -0.25f, -0.5f, -0.25f };

        std::vector<std::complex<float>> ref;
        {
            float acc = 0.0f;
            auto sc = [](float ph){ return std::complex<float>(std::cos(ph),
                                                               std::sin(ph)); };
            for (float x : src_data) {
                acc = std::remainder(acc + k * x,
                                     2.0f * std::numbers::pi_v<float>);
                ref.push_back(sc(acc));
            }
        }

        auto out = run_pipeline<float, std::complex<float>>(
                       { std::begin(src_data), std::end(src_data) },
                       { { "sensitivity", k } });

        expect(out.size() == ref.size());
        for (std::size_t i = 0; i < ref.size(); ++i)
            expect( std::abs(out[i] - ref[i]) < 1e-5f ) << "i=" << i;
    };
};

int main() { }   // Boost.UT auto-runs suites
