#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/equalizer/AdaptiveAlgorithm.hpp>

#include <vector>
#include <complex>
#include <random>
#include <cmath>
#include <cstddef>
#include <algorithm>

using namespace boost::ut;
using gr::digital::AdaptiveEQCF;
using gr::digital::AdaptAlg;

namespace {

using cfloat = std::complex<float>;

static std::vector<int> rand_bits(std::size_t n, unsigned seed=123)
{
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> b(0,1);
    std::vector<int> v(n);
    for (auto& x : v) x = b(rng);
    return v;
}

static std::vector<cfloat> bpsk_on_I(const std::vector<int>& bits)
{
    std::vector<cfloat> s; s.reserve(bits.size());
    for (auto b : bits) s.emplace_back((b ? 1.f : -1.f), 0.f);
    return s;
}

static std::vector<cfloat> apply_channel(const std::vector<cfloat>& s,
                                         const std::vector<cfloat>& h,
                                         float noise_sigma=0.f,
                                         unsigned seed=7)
{
    std::mt19937 rng(seed);
    std::normal_distribution<float> N(0.f, noise_sigma);

    const std::size_t L = h.size();
    std::vector<cfloat> y(s.size(), cfloat{0.f,0.f});
    for (std::size_t n=0; n<s.size(); ++n) {
        cfloat acc{0.f,0.f};
        for (std::size_t k=0; k<L; ++k) {
            if (n>=k) acc += s[n-k] * h[k];
        }
        if (noise_sigma > 0.f)
            acc += cfloat{N(rng), N(rng)};
        y[n] = acc;
    }
    return y;
}

static float mse(const std::vector<cfloat>& y, const std::vector<cfloat>& ref, std::size_t from)
{
    float acc = 0.0f;
    std::size_t cnt = 0;
    for (std::size_t i = from; i < y.size() && i < ref.size(); ++i) {
        const float e2 = std::norm(y[i] - ref[i]);
        acc += e2; ++cnt;
    }
    return (cnt == 0) ? 0.f : (acc / static_cast<float>(cnt));
}

} // namespace

const suite AdaptiveAlgorithmSuite = [] {
    "LMS: training then DD reduces MSE"_test = [] {
        const std::size_t N = 4000;
        const auto bits = rand_bits(N);
        const auto s    = bpsk_on_I(bits);

        const std::vector<cfloat> h = { {0.9f,0.0f}, {0.3f,0.2f}, {-0.15f,0.0f} };
        const auto x = apply_channel(s, h, /*noise_sigma*/0.02f, /*seed*/1);

        AdaptiveEQCF eq; eq.start(/*L*/7, /*mu*/0.02f, AdaptAlg::LMS);

        const std::size_t Nt = 1200;
        std::vector<cfloat> y(N);
        for (std::size_t n=0; n<Nt; ++n)
            eq.processOneTrain(x[n], s[n], y[n]);

        for (std::size_t n=Nt; n<N; ++n)
            eq.processOneDD(x[n], y[n]);

        const float mse_train = mse(y, s, 0);
        const float mse_dd    = mse(y, s, Nt);
        expect(mse_dd < 0.3f && mse_train < 0.8f);
    };

    "NLMS: training then DD reduces MSE"_test = [] {
        const std::size_t N = 4000;
        const auto bits = rand_bits(N, 77);
        const auto s    = bpsk_on_I(bits);

        const std::vector<cfloat> h = { {1.0f,0.0f}, {0.2f,0.15f}, {-0.1f,0.0f} };
        const auto x = apply_channel(s, h, 0.03f, 5);

        AdaptiveEQCF eq; eq.start(/*L*/9, /*mu*/0.5f, AdaptAlg::NLMS);

        const std::size_t Nt = 1500;
        std::vector<cfloat> y(N);
        for (std::size_t n=0; n<Nt; ++n)
            eq.processOneTrain(x[n], s[n], y[n]);
        for (std::size_t n=Nt; n<N; ++n)
            eq.processOneDD(x[n], y[n]);

        const float mse_dd = mse(y, s, Nt);
        expect(mse_dd < 0.35f);
    };

    "CMA: blind adaptation reduces output variance from target"_test = [] {
        const std::size_t N = 5000;
        const auto bits = rand_bits(N, 999);
        const auto s    = bpsk_on_I(bits);

        const std::vector<cfloat> h = { {0.8f,0.0f}, {0.25f,0.2f}, {-0.12f,0.0f} };
        const auto x = apply_channel(s, h, 0.02f, 23);

        AdaptiveEQCF eq; eq.start(/*L*/11, /*mu*/0.0008f, AdaptAlg::CMA, /*R*/1.0f);

        std::vector<cfloat> y(N);
        for (std::size_t n=0; n<N; ++n)
            eq.processOneDD(x[n], y[n]);

        float acc = 0.0f; std::size_t cnt = 0;
        for (std::size_t n=N/2; n<N; ++n) {
            const float e = std::norm(y[n]) - 1.0f;
            acc += std::abs(e); ++cnt;
        }
        const float mean_abs_dev = (cnt == 0) ? 0.f : (acc / static_cast<float>(cnt));
        expect(mean_abs_dev < 0.3f);
    };
};

int main() {}
