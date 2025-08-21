#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/ofdm/CarrierAllocator.hpp>

#include <vector>
#include <complex>
#include <cstddef>
#include <iostream>

using namespace boost::ut;
using gr::digital::OfdmCarrierAllocatorCVC;
using cfloat = std::complex<float>;

static std::vector<cfloat> to_cplx(const std::vector<float>& v) {
    std::vector<cfloat> out; out.reserve(v.size());
    for (float x : v) out.emplace_back(x, 0.0f);
    return out;
}
static std::vector<cfloat> to_cplxj(const std::vector<int>& v_im) {
    std::vector<cfloat> out; out.reserve(v_im.size());
    for (int k : v_im) out.emplace_back(0.0f, static_cast<float>(k));
    return out;
}

static void print_first_mismatch(const std::vector<cfloat>& flat,
                                 const std::vector<cfloat>& expected,
                                 std::size_t fft_len) {
    for (std::size_t i = 0; i < expected.size() && i < flat.size(); ++i) {
        if (flat[i] != expected[i]) {
            const std::size_t sym = i / fft_len;
            const std::size_t bin = i % fft_len;
            std::cerr << "[Allocator] mismatch at flat[" << i << "]  (sym " << sym
                      << ", bin " << bin << "):  got (" << flat[i].real() << "," << flat[i].imag()
                      << "), expected (" << expected[i].real() << "," << expected[i].imag() << ")\n";
            break;
        }
    }
}

const suite CarrierAllocatorSuite = [] {
    "simple with sync word (shifted output)"_test = [] {
        const std::size_t N = 6;
        const std::vector<float> tx_r = {1,2,3};
        const std::vector<int> pilot_im = {1}; // 1j
        const std::vector<float> sync_r = {0,1,2,3,4,5};
        OfdmCarrierAllocatorCVC alloc;
        alloc.start(
            N,
            /*occupied*/{{0,1,2}},
            /*pilot carriers*/{{3}},
            /*pilot symbols*/{ to_cplxj(pilot_im) },
            /*sync words*/{ to_cplx(sync_r) },
            /*shifted*/true
        );

        std::vector<std::vector<cfloat>> out;
        alloc.map_frame(to_cplx(tx_r), out);
        expect(out.size() == 2u); // 1 sync + 1 data symbol

        std::vector<cfloat> expected0 = to_cplx(sync_r);
        std::vector<cfloat> expected1 = { {0,1}, {0,0}, {0,0}, {1,0}, {2,0}, {3,0} };
        expect(out[0] == expected0);
        expect(out[1] == expected1);
    };

    "odd N, negative pilot index (shifted output)"_test = [] {
        const std::size_t N = 5;
        const std::vector<float> tx_r = {1,2,3};
        const std::vector<int> pilot_im = {1}; // 1j
        OfdmCarrierAllocatorCVC alloc;
        alloc.start(
            N,
            /*occupied*/{{0,1,2}},
            /*pilot carriers*/{{-2}},
            /*pilot symbols*/{ to_cplxj(pilot_im) },
            /*sync words*/{},
            /*shifted*/true
        );

        std::vector<std::vector<cfloat>> out;
        alloc.map_frame(to_cplx(tx_r), out);
        expect(out.size() == 1u);

        std::vector<cfloat> expected = { {0,1}, {0,0}, {1,0}, {2,0}, {3,0} };
        expect(out[0] == expected);
    };

    "negative occupied, pilot at +3 (shifted)"_test = [] {
        const std::size_t N = 6;
        const std::vector<float> tx_r = {1,2,3};
        const std::vector<int> pilot_im = {1}; // 1j
        OfdmCarrierAllocatorCVC alloc;
        alloc.start(
            N,
            /*occupied*/{{-1, 1, 2}},
            /*pilot carriers*/{{3}},
            /*pilot symbols*/{ to_cplxj(pilot_im) },
            /*sync words*/{},
            /*shifted*/true
        );

        std::vector<std::vector<cfloat>> out;
        alloc.map_frame(to_cplx(tx_r), out);
        expect(out.size() == 1u);

        std::vector<cfloat> expected = { {0,1}, {0,0}, {1,0}, {0,0}, {2,0}, {3,0} };
        expect(out[0] == expected);
    };

    "with sync word and two OFDM symbols (shifted)"_test = [] {
        const std::size_t N = 6;
        const std::vector<float> tx_r = {1,2,3,4,5,6};
        const std::vector<int> pilot_im = {1}; // 1j
        const std::vector<float> sync_r(N, 0.f);
        OfdmCarrierAllocatorCVC alloc;
        alloc.start(
            N,
            /*occupied*/{{-1, 1, 2}},
            /*pilot carriers*/{{3}},
            /*pilot symbols*/{ to_cplxj(pilot_im) },
            /*sync words*/{ to_cplx(sync_r) },
            /*shifted*/true
        );

        std::vector<std::vector<cfloat>> out;
        alloc.map_frame(to_cplx(tx_r), out);
        expect(out.size() == 3u); // 1 sync + 2 data symbols

        std::vector<cfloat> expected0 = to_cplx(sync_r);
        std::vector<cfloat> expected1 = { {0,1}, {0,0}, {1,0}, {0,0}, {2,0}, {3,0} };
        std::vector<cfloat> expected2 = { {0,1}, {0,0}, {4,0}, {0,0}, {5,0}, {6,0} };
        expect(out[0] == expected0);
        expect(out[1] == expected1);
        expect(out[2] == expected2);
    };

    "advanced: pilots & multiple sets (unshifted)"_test = [] {
        const std::size_t N = 16;
        std::vector<float> data_r; data_r.reserve(15);
        for (int i=1; i<=15; ++i) data_r.push_back(static_cast<float>(i));

        OfdmCarrierAllocatorCVC alloc;
        alloc.start(
            N,
            /*occupied*/{
                {1,3,4,11,12,14},
                {1,2,4,11,13,14}
            },
            /*pilot carriers*/{
                {2,13},
                {3,12}
            },
            /*pilot symbols*/{
                { cfloat{0,1}, cfloat{0,2} }, // (1j, 2j)
                { cfloat{0,3}, cfloat{0,4} }  // (3j, 4j)
            },
            /*sync words*/{},
            /*shifted*/false
        );

        std::vector<std::vector<cfloat>> out_syms;
        alloc.map_frame(to_cplx(data_r), out_syms);
        expect(out_syms.size() == 3u);

        std::vector<cfloat> flat;
        for (std::size_t k=0; k<3; ++k)
            flat.insert(flat.end(), out_syms[k].begin(), out_syms[k].end());


        std::vector<cfloat> expected = {
            {0,0},{1,0},{0,1},{2,0},{3,0},{0,0},{0,0},{0,0},
            {0,0},{0,0},{0,0},{4,0},{5,0},{0,2},{6,0},{0,0},
            {0,0},{7,0},{8,0},{0,3},{9,0},{0,0},{0,0},{0,0},
            {0,0},{0,0},{0,0},{10,0},{0,4},{11,0},{12,0},{0,0},
            {0,0},{13,0},{0,1},{14,0},{15,0},{0,0},{0,0},{0,0},
            {0,0},{0,0},{0,0},{0,0},{0,0},{0,2},{0,0},{0,0}
        };

        if (flat != expected) {
            print_first_mismatch(flat, expected, N);
        }
        expect(flat == expected);
    };
};
