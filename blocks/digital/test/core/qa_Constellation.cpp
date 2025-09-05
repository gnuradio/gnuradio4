#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/core/Constellation.hpp>

#include <cmath>
#include <limits>

using namespace boost::ut;
using gr::digital::BPSK;
using gr::digital::QPSK_Gray;
using gr::digital::QAM16_Gray;
using gr::digital::Normalization;
using Slice = gr::digital::EuclideanSlicer;
using cfloat = gr::digital::cfloat;

const suite ConstellationSuite = [] {
    "BPSK hard slice"_test = [] {
        constexpr auto c = BPSK(); // raw
        expect(Slice::processOneLabel(c, cfloat{-0.7f, 0.f}) == 0u);
        expect(Slice::processOneLabel(c, cfloat{+0.2f, 0.f}) == 1u);
    };

    "QPSK Gray hard slice"_test = [] {
        constexpr auto c = QPSK_Gray(); // raw (scale doesn't affect decisions)
        expect(Slice::processOneLabel(c, cfloat{-0.6f, +0.9f}) == 2u); // (-,+) -> 2
        expect(Slice::processOneLabel(c, cfloat{+0.4f, -0.2f}) == 1u); // (+,-) -> 1
        expect(Slice::processOneLabel(c, cfloat{+0.9f, +0.9f}) == 3u);
        expect(Slice::processOneLabel(c, cfloat{-0.9f, -0.9f}) == 0u);
    };

    "QAM16 Gray hard slice"_test = [] {
        constexpr auto c = QAM16_Gray();
        expect(Slice::processOneLabel(c, cfloat{3.1f, 2.9f}) == 0xAu);
        expect(Slice::processOneLabel(c, cfloat{-2.8f, -3.2f}) == 0x0u);
        expect(Slice::processOneLabel(c, cfloat{1.05f, -0.9f}) == 0xDu);
    };

    // Tie-breaking: equidistant from (-1,-1) and (-1,+1) at (-1,0) -> lowest index
    "Tie-break is stable (lowest index)"_test = [] {
        constexpr auto c = QPSK_Gray();
        const auto idx = Slice::processOneIndex(c, cfloat{-1.f, 0.f});
        expect(idx == 0u); // (-1,-1) is index 0 in our table
    };

    // Corner cases: NaN/Inf
    "Corner: non-finite sample -> first label"_test = [] {
        constexpr auto c = QPSK_Gray();
        const float nan = std::numeric_limits<float>::quiet_NaN();
        const float inf = std::numeric_limits<float>::infinity();
        expect(Slice::processOneLabel(c, cfloat{nan, 0.f}) == c.labels[0]);
        expect(Slice::processOneLabel(c, cfloat{0.f, inf}) == c.labels[0]);
    };

    "Normalization: power"_test = [] {
        auto c = QPSK_Gray().normalized(Normalization::Power);
        float s = 0.f;
        for (auto z : c.points) s += std::norm(z);
        expect(std::abs(s / 4.0f - 1.0f) < 1e-6f);
    };

    "Normalization: amplitude"_test = [] {
        auto c = QAM16_Gray().normalized(Normalization::Amplitude);
        float s = 0.f;
        for (auto z : c.points) s += std::abs(z);
        expect(std::abs(s / 16.0f - 1.0f) < 1e-6f);
    };
};
