#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/ofdm/CyclicPrefixer.hpp>

#include <vector>
#include <complex>

using namespace boost::ut;
using gr::digital::OfdmCyclicPrefixerCF;
using cfloat = std::complex<float>;

static std::vector<cfloat> seqi(int first, int last_inclusive) {
    std::vector<cfloat> v;
    for (int i = first; i <= last_inclusive; ++i) v.emplace_back(static_cast<float>(i), 0.0f);
    return v;
}

const suite CyclicPrefixerSuite = [] {
    "wo_tags_no_rolloff (uniform CP)"_test = [] {
        const int fft_len = 8, cp_len = 2;
        OfdmCyclicPrefixerCF cp;
        cp.start(fft_len, cp_len, /*rolloff*/0, /*framed*/false);

        std::vector<cfloat> out;
        const auto s = seqi(0, 7);
        cp.processOne(s.data(), out);
        cp.processOne(s.data(), out);

        std::vector<cfloat> exp;
        exp.push_back({6,0}); exp.push_back({7,0});
        for (int i=0;i<8;++i) exp.emplace_back(i,0);
        exp.push_back({6,0}); exp.push_back({7,0});
        for (int i=0;i<8;++i) exp.emplace_back(i,0);

        expect(out.size() == exp.size());
        for (std::size_t i=0;i<out.size();++i) expect(out[i] == exp[i]);
    };

    "wo_tags_rolloff2 (uniform CP)"_test = [] {
        const int fft_len = 8, cp_len = 2;
        OfdmCyclicPrefixerCF cp;
        cp.start(fft_len, cp_len, /*rolloff*/2, /*framed*/false);

        std::vector<cfloat> out;
        const auto s = seqi(1, 8); 
        cp.processOne(s.data(), out);
        cp.processOne(s.data(), out);

        std::vector<cfloat> exp;
        exp.emplace_back(7.0f/2.0f, 0.0f); exp.emplace_back(8,0);
        for (int i=1;i<=8;++i) exp.emplace_back(i,0);
        exp.emplace_back(7.0f/2.0f + 1.0f/2.0f, 0.0f); exp.emplace_back(8,0);
        for (int i=1;i<=8;++i) exp.emplace_back(i,0);

        expect(out.size() == exp.size());
        for (std::size_t i=0;i<out.size();++i) expect(out[i] == exp[i]);
    };

    "multiple_cps_no_rolloff"_test = [] {
        const int fft_len = 8;
        const std::vector<int> cps{3,2,2};
        OfdmCyclicPrefixerCF cp;
        cp.start(fft_len, cps, /*rolloff*/0, /*framed*/false);

        std::vector<cfloat> out;
        const auto s0 = seqi(0,7);
        cp.processOne(s0.data(), out);
        cp.processOne(s0.data(), out);
        cp.processOne(s0.data(), out);
        cp.processOne(s0.data(), out);
        cp.processOne(s0.data(), out);

        // expected: exactly the GR3 sequence in test_wo_tags_no_rolloff_multiple_cps
        std::vector<cfloat> exp = {
            {5,0},{6,0},{7,0}, {0,0},{1,0},{2,0},{3,0},{4,0},{5,0},{6,0},{7,0},
            {6,0},{7,0},       {0,0},{1,0},{2,0},{3,0},{4,0},{5,0},{6,0},{7,0},
            {6,0},{7,0},       {0,0},{1,0},{2,0},{3,0},{4,0},{5,0},{6,0},{7,0},
            {5,0},{6,0},{7,0}, {0,0},{1,0},{2,0},{3,0},{4,0},{5,0},{6,0},{7,0},
            {6,0},{7,0},       {0,0},{1,0},{2,0},{3,0},{4,0},{5,0},{6,0},{7,0},
        };
        expect(out.size() == exp.size());
        for (std::size_t i=0;i<out.size();++i) expect(out[i] == exp[i]);
    };

    "framed_finalize_emits_tail"_test = [] {
        const int fft_len = 8, cp_len = 2;
        OfdmCyclicPrefixerCF cp;
        cp.start(fft_len, cp_len, /*rolloff*/2, /*framed*/true);

        std::vector<cfloat> out;
        const auto s = seqi(1, 8);
        cp.processOne(s.data(), out);
        cp.finalize(out);

        // expected: first symbol with rolloff start (7/2,8,1..8) + trailing 0.5*first_sample (1/2)
        std::vector<cfloat> exp;
        exp.emplace_back(7.0f/2.0f, 0.0f); exp.emplace_back(8,0);
        for (int i=1;i<=8;++i) exp.emplace_back(i,0);
        exp.emplace_back(1.0f/2.0f, 0.0f); // tail
        expect(out.size() == exp.size());
        for (std::size_t i=0;i<out.size();++i) expect(out[i] == exp[i]);
    };
};

int main() {}
