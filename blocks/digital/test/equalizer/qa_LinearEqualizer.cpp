#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/equalizer/LinearEqualizer.hpp>

#include <vector>
#include <complex>
#include <random>
#include <cmath>
#include <cstddef>

using namespace boost::ut;
using gr::digital::LinearEqualizerCF;
using gr::digital::AdaptAlg;

namespace {

using cfloat = std::complex<float>;

static std::vector<int> rand_bits(std::size_t n, unsigned seed=777)
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
                                         unsigned seed=11)
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

const suite LinearEqSuite = [] {
    "LinearEqualizer: LMS training then DD, MSE < 0.3 after converge"_test = [] {
        const std::size_t N  = 4000;
        const std::size_t Nt = 1000;   // training symbols
        const auto bits = rand_bits(N, 123);
        const auto s    = bpsk_I(bits);

        const std::vector<cfloat> h = { {0.9f,0.0f}, {0.25f,0.15f}, {-0.1f,0.0f} };
        const auto x = apply_channel(s, h, 0.02f, 5);

        std::vector<cfloat> train(s.begin(), s.begin()+Nt);

        LinearEqualizerCF leq;
        leq.start(/*L*/11, /*sps*/1, AdaptAlg::LMS, /*mu*/0.02f, /*R*/1.0f,
                  /*adapt_after_training*/true, train);

        std::vector<cfloat> y(N);
        const std::vector<unsigned> tstarts{0u};
        const std::size_t outN = leq.equalize(x.data(), y.data(), N, N, tstarts);

        expect(outN == N);

        const float mse_tail = mse(y, s, Nt + 500); 
        expect(mse_tail < 0.3f);
    };
};
