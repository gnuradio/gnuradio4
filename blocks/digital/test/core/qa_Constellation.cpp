#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/core/Constellation.hpp>

#include <vector>
#include <cmath>

using namespace boost::ut;
using gr::digital::BPSK;
using gr::digital::QPSK_Gray;
using gr::digital::QAM16_Gray;
using gr::digital::Normalization;
using gr::digital::slice_label_euclidean;
using cfloat = gr::digital::cfloat;

const suite ConstellationSuite = [] {
    "BPSK hard slice"_test = [] {
        auto c = BPSK(); // raw
        expect(slice_label_euclidean(c, cfloat{-0.7f, 0.f}) == 0u);
        expect(slice_label_euclidean(c, cfloat{+0.2f, 0.f}) == 1u);
    };

    "QPSK Gray hard slice"_test = [] {
        auto c = QPSK_Gray(); // raw (scale doesn't affect decisions)
        expect(slice_label_euclidean(c, cfloat{-0.6f, +0.9f}) == 2u); // (-,+) -> 2
        expect(slice_label_euclidean(c, cfloat{+0.4f, -0.2f}) == 1u); // (+,-) -> 1
        expect(slice_label_euclidean(c, cfloat{+0.9f, +0.9f}) == 3u);
        expect(slice_label_euclidean(c, cfloat{-0.9f, -0.9f}) == 0u);
    };

    "QAM16 Gray hard slice"_test = [] {
        auto c = QAM16_Gray();
        // near (+3,+3) -> label 0xA (per GR3 mapping table)
        expect(slice_label_euclidean(c, cfloat{3.1f, 2.9f}) == 0xAu);
        // near (-3,-3) -> 0x0
        expect(slice_label_euclidean(c, cfloat{-2.8f, -3.2f}) == 0x0u);
        // near (+1,-1) -> 0xD
        expect(slice_label_euclidean(c, cfloat{1.05f, -0.9f}) == 0xDu);
    };

    "Normalization: power"_test = [] {
        auto c = QPSK_Gray().normalized(Normalization::Power);
        // Mean power should be ~1
        float s = 0.f;
        for (auto z : c.points) s += std::norm(z);
        expect(std::abs(s / 4.f - 1.0f) < 1e-6f);
    };

    "Normalization: amplitude"_test = [] {
        auto c = QAM16_Gray().normalized(Normalization::Amplitude);
        float s = 0.f;
        for (auto z : c.points) s += std::abs(z);
        expect(std::abs(s / 16.f - 1.0f) < 1e-6f);
    };
};
