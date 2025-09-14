#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/ofdm/Serializer.hpp>

#include <vector>
#include <complex>
#include <cstddef>

using namespace boost::ut;
using gr::digital::OfdmSerializerVCC;
using cfloat = std::complex<float>;

static std::vector<cfloat> to_cplx(const std::vector<float>& v) {
    std::vector<cfloat> out; out.reserve(v.size());
    for (float x : v) out.emplace_back(x, 0.0f);
    return out;
}

const suite OfdmSerializerSuite = [] {
    "simple (unshifted)"_test = [] {
        const std::size_t fft_len = 16;
        const std::vector<float> txr = {
            0,1,0,2,3,0,0,0, 0,0,0,4,5,0,6,0,
            0,7,8,0,9,0,0,0, 0,0,0,10,0,11,12,0,
            0,13,0,14,15,0,0,0, 0,0,0,0,0,0,0,0
        };
        std::vector<cfloat> tx = to_cplx(txr);

        const std::vector<std::vector<int>> occ = {
            {1,3,4,11,12,14},
            {1,2,4,11,13,14}
        };

        OfdmSerializerVCC ser;
        ser.start(fft_len, occ, /*input_is_shifted=*/false);
        std::vector<cfloat> out;
        ser.processSymbols(tx.data(), /*n_syms=*/3, out);

        std::vector<float> ex;
        for (int i = 1; i <= 15; ++i) ex.push_back(static_cast<float>(i));
        ex.push_back(0.f); ex.push_back(0.f); ex.push_back(0.f);

        expect(out.size() == ex.size());
        for (std::size_t i = 0; i < ex.size(); ++i)
            expect(out[i].real() == ex[i] && out[i].imag() == 0.0f);
    };

    "shifted (FFT-shifted bins with negative indices)"_test = [] {
        const std::size_t fft_len = 16;
        const std::vector<float> txr = {
            0,0,0,0,0,0, 1,2, 0,3,4,5, 0,0,0,0,
            0,0,0,0, 6,0,7,8, 0,9,10,0, 11,0,0,0,
            0,0,0,0, 0,12,13,14, 0,15,16,17, 0,0,0,0
        };
        std::vector<cfloat> tx = to_cplx(txr);

        const std::vector<std::vector<int>> occ = {
            {13,14,15, 1,2,3},
            {-4,-2,-1, 1,2,4}
        };

        OfdmSerializerVCC ser;
        ser.start(fft_len, occ, /*input_is_shifted=*/true);
        std::vector<cfloat> out;
        ser.processSymbols(tx.data(), /*n_syms=*/3, out);

        for (std::size_t i = 0; i < 18u; ++i)
            expect(out[i].real() == static_cast<float>(i) && out[i].imag() == 0.0f);
    };

    "with carrier offset (unshifted)"_test = [] {
        const std::size_t fft_len = 16;


        const std::vector<cfloat> tx = {
            {0,0}, {0,0}, {1,0}, {0,1}, {2,0}, {3,0}, {0,0}, {0,0},
            {0,0}, {0,0}, {0,0}, {0,0}, {4,0}, {5,0}, {0,2}, {6,0},
            {0,0}, {0,0}, {7,0}, {8,0}, {0,3}, {9,0}, {0,0}, {0,0},
            {0,0}, {0,0}, {0,0}, {0,0}, {10,0}, {0,4}, {11,0}, {12,0},
            {0,0}, {0,0}, {13,0}, {0,1}, {14,0}, {15,0}, {0,0}, {0,0},
            {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,2}, {0,0}
        };

        const std::vector<std::vector<int>> occ = {
            {1,3,4,11,12,14},
            {1,2,4,11,13,14}
        };

        OfdmSerializerVCC ser;
        ser.start(fft_len, occ, /*input_is_shifted=*/false);
        ser.set_carrier_offset(1); 

        std::vector<cfloat> out;
        ser.processSymbols(tx.data(), /*n_syms=*/3, out);

        std::vector<float> ex;
        for (int i = 1; i <= 15; ++i) ex.push_back(static_cast<float>(i));
        ex.push_back(0.f); ex.push_back(0.f); ex.push_back(0.f);

        expect(out.size() == ex.size());
        for (std::size_t i = 0; i < ex.size(); ++i)
            expect(out[i].real() == ex[i] && out[i].imag() == 0.0f);
    };
};
