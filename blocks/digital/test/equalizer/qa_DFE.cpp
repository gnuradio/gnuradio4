#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/equalizer/DecisionFeedbackEqualizer.hpp>

#include <vector>
#include <complex>
#include <random>
#include <cmath>
#include <cstddef>

using namespace boost::ut;
using gr::digital::DecisionFeedbackEqualizerCF;
using gr::digital::DfeAlg;

namespace {

using cfloat = std::complex<float>;

static std::vector<int> rand_bits(std::size_t n, unsigned seed=135)
{
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> b(0,1);
    std::vector<int> v(n);
    for (auto& x : v) x = b(rng);
    return v;
}
static std::vector<cfloat> bpsk_I(const std::vector<int>& bits)
{
    std::vector<cfloat> s; s.reserve(bits.size());
    for (auto b : bits) s.emplace_back((b ? 1.f : -1.f), 0.f);
    return s;
}
static std::vector<cfloat> apply_channel(const std::vector<cfloat>& s,
                                         const std::vector<cfloat>& h,
                                         float noise_sigma=0.f,
                                         unsigned seed=77)
{
    std::mt19937 rng(seed);
    std::normal_distribution<float> N(0.f, noise_sigma);
    const std::size_t L = h.size();
    std::vector<cfloat> y(s.size(), cfloat{0.f,0.f});
    for (std::size_t n = 0; n < s.size(); ++n) {
        cfloat acc{0.f,0.f};
        for (std::size_t k = 0; k < L; ++k) if (n >= k) acc += s[n-k] * h[k];
        if (noise_sigma > 0.f) acc += cfloat{N(rng), N(rng)};
        y[n] = acc;
    }
    return y;
}
static float mse(const std::vector<cfloat>& y, const std::vector<cfloat>& ref, std::size_t from)
{
    float acc = 0.0f; std::size_t cnt = 0;
    for (std::size_t i = from; i < y.size() && i < ref.size(); ++i) {
        acc += std::norm(y[i] - ref[i]); ++cnt;
    }
    return (cnt == 0) ? 0.f : (acc / static_cast<float>(cnt));
}

} // namespace

const suite DFESuite = [] {
    "DFE: LMS training then DD, tail MSE < 0.3"_test = [] {
        const std::size_t N  = 4000;
        const std::size_t Nt = 800;     // training symbols
        const auto bits = rand_bits(N, 4242);
        const auto s    = bpsk_I(bits);

        const std::vector<cfloat> h = { {0.85f,0.f}, {0.25f,0.15f}, {-0.12f,0.0f} };
        const auto x = apply_channel(s, h, 0.02f, 9);

        std::vector<cfloat> train(s.begin(), s.begin()+Nt);

        DecisionFeedbackEqualizerCF dfe;
        dfe.start(/*Lf*/11, /*Lb*/3, /*sps*/1, DfeAlg::LMS,
                  /*mu_f*/0.02f, /*mu_b*/0.02f, /*R*/1.0f,
                  /*adapt_after_training*/true, train);

        std::vector<cfloat> y(N);
        const std::vector<unsigned> tstarts{0u};
        const auto outN = dfe.equalize(x.data(), y.data(), N, N, tstarts);
        expect(outN == N);

        const float tail = mse(y, s, Nt + 400);
        expect(tail < 0.3f);
    };
};
