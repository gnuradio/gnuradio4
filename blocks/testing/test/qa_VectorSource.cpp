// qa_VectorSource.cpp — per-block functional tests
#include <boost/ut.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/VectorSink.hpp>
#include <gnuradio-4.0/testing/VectorSource.hpp>

using namespace boost::ut;

const boost::ut::suite<"VectorSource"> vectorSourceTests = [] {
    using namespace boost::ut;
    "replays data in order"_test = [] {
        const std::vector<float> inputVec = {10.f, 20.f, 30.f};
        gr::Graph                graph;
        auto&                    src = graph.emplaceBlock<gr::testing::VectorSource<float>>({{"data", inputVec}});
        auto&                    snk = graph.emplaceBlock<gr::testing::VectorSink<float>>({});
        expect(graph.connect<"out", "in">(src, snk).has_value());
        gr::scheduler::Simple sched;
        expect(sched.exchange(std::move(graph)).has_value());
        expect(sched.runAndWait().has_value());
        const auto& out = snk.data();
        expect(ge(out.size(), inputVec.size()));
        expect(approx(out[0], 10.f, 1e-6f));
        expect(approx(out[1], 20.f, 1e-6f));
        expect(approx(out[2], 30.f, 1e-6f));
    };
};

const boost::ut::suite<"VectorSource graph"> vectorSourceGraphTests = [] {
    using namespace boost::ut;

    "graph: data passes through VectorSource → VectorSink"_test = [] {
        const std::vector<float> inputVec = {10.f, 20.f, 30.f, 40.f};

        gr::Graph graph;
        auto&     src = graph.emplaceBlock<gr::testing::VectorSource<float>>({{"data", inputVec}});
        auto&     snk = graph.emplaceBlock<gr::testing::VectorSink<float>>({});
        expect(graph.connect<"out", "in">(src, snk).has_value());

        gr::scheduler::Simple sched;
        expect(sched.exchange(std::move(graph)).has_value());
        expect(sched.runAndWait().has_value());

        const auto& out = snk.data();
        expect(ge(out.size(), inputVec.size()));
        for (std::size_t i = 0; i < inputVec.size(); ++i) {
            expect(approx(out[i], inputVec[i], 1e-6f));
        }
    };

    "graph: uint8_t data passes through"_test = [] {
        const std::vector<uint8_t> inputVec = {1u, 2u, 3u, 255u};

        gr::Graph graph;
        auto&     src = graph.emplaceBlock<gr::testing::VectorSource<uint8_t>>({{"data", inputVec}});
        auto&     snk = graph.emplaceBlock<gr::testing::VectorSink<uint8_t>>({});
        expect(graph.connect<"out", "in">(src, snk).has_value());

        gr::scheduler::Simple sched;
        expect(sched.exchange(std::move(graph)).has_value());
        expect(sched.runAndWait().has_value());

        const auto& out = snk.data();
        expect(ge(out.size(), inputVec.size()));
        for (std::size_t i = 0; i < inputVec.size(); ++i) {
            expect(eq(out[i], inputVec[i]));
        }
    };
};

int main() {}
