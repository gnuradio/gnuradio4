// qa_VectorSink.cpp — per-block functional tests
#include <boost/ut.hpp>
#include <gnuradio-4.0/testing/VectorSink.hpp>
#include <gnuradio-4.0/testing/VectorSource.hpp>
using namespace boost::ut;

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

const boost::ut::suite<"VectorSink"> vectorSinkTests = [] {
    using namespace boost::ut;
    "accumulates samples"_test = [] {
        gr::testing::VectorSink<float> sink;
        sink.processOne(1.f);
        sink.processOne(2.f);
        sink.processOne(3.f);
        expect(eq(sink.data().size(), std::size_t{3}));
        expect(approx(sink.data()[2], 3.f, 1e-6f));
    };
};

const boost::ut::suite<"VectorSink graph"> vectorSinkGraphTests = [] {
    using namespace boost::ut;

    "graph: VectorSource → VectorSink collects all samples"_test = [] {
        const std::vector<float> inputVec = {1.f, 2.f, 3.f, 4.f, 5.f};

        gr::Graph graph;
        auto&     src = graph.emplaceBlock<gr::testing::VectorSource<float>>({{"data", inputVec}});
        auto&     snk = graph.emplaceBlock<gr::testing::VectorSink<float>>({});
        expect(graph.connect<"out", "in">(src, snk).has_value());

        gr::scheduler::Simple sched;
        expect(sched.exchange(std::move(graph)).has_value());
        expect(sched.runAndWait().has_value());

        const auto& out = snk.data();
        expect(ge(out.size(), inputVec.size()));
        expect(approx(out[0], 1.f, 1e-5f));
        expect(approx(out[4], 5.f, 1e-5f));
    };
};

int main() {}
