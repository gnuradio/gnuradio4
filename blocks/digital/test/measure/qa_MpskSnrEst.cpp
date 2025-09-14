#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/measure/MpskSnrEst.hpp>

#include <vector>
#include <complex>
#include <random>
#include <cmath>
#include <cstddef>
#include <algorithm>

using namespace boost::ut;
using gr::digital::MpskSnrM2M4;
using gr::digital::MpskSnrSimple;

static std::vector<std::complex<float>> bpsk_awgn(std::size_t n, float sigma, unsigned seed = 123)
{
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> bit(0,1);
    std::normal_distribution<float>    N(0.0f, sigma);

    std::vector<std::complex<float>> v;
    v.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const float s = bit(rng) ? +1.0f : -1.0f;   // BPSK on I
        v.emplace_back(s + N(rng), N(rng));
    }
    return v;
}

static inline double theor_snr_linear(float sigma) {
    const double Ps = 1.0;
    const double Pn = 2.0 * static_cast<double>(sigma) * static_cast<double>(sigma);
    return Ps / std::max(Pn, 1e-30);
}
static inline double to_db(double x) { return 10.0 * std::log10(std::max(x, 1e-30)); }

const suite MpskSnrEstSuite = [] {
    "M2M4: BPSK AWGN — estimate tracks theory within ~1 dB"_test = [] {
        for (float sigma : {0.05f, 0.1f, 0.2f}) {
            const auto sig = bpsk_awgn(200000, sigma, 7);

            MpskSnrM2M4 est; est.start(0.001);
            for (const auto& x : sig) est.processOne(x);

            const double snr_est = est.snr_db();
            const double snr_th  = to_db(theor_snr_linear(sigma));
            expect(std::abs(snr_est - snr_th) < 1.0_d);
        }
    };

    "Simple: monotone decreasing vs noise level"_test = [] {
        std::vector<double> estimates;
        for (float sigma : {0.05f, 0.1f, 0.2f, 0.3f}) {
            const auto sig = bpsk_awgn(120000, sigma, 9);
            MpskSnrSimple est; est.start(0.003, MpskSnrSimple::SimpleMode::BPSK_I);
            for (const auto& x : sig) est.processOne(x);
            estimates.push_back(est.snr_db());
        }
        bool mono = true;
        for (std::size_t i = 1; i < estimates.size(); ++i)
            if (!(estimates[i] < estimates[i-1])) { mono = false; break; }
        expect(mono) << "simple estimator should be monotone vs noise level";
    };
};

int main() {}