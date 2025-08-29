#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/timing/FllBandEdge.hpp>

#include <vector>
#include <complex>
#include <random>
#include <cmath>
#include <cstddef>

using namespace boost::ut;

namespace {

constexpr float kPi = 3.14159265358979323846f;

static std::vector<std::complex<float>> tone(std::size_t n, float omega)
{
    std::vector<std::complex<float>> v;
    v.reserve(n);
    float ph = 0.0f;
    for (std::size_t i = 0; i < n; ++i) {
        v.emplace_back(std::cos(ph), std::sin(ph));
        ph += omega;
        if (ph >  kPi) ph -= 2.0f * kPi;
        if (ph < -kPi) ph += 2.0f * kPi;
    }
    return v;
}

} // namespace

const suite FllBandEdgeSuite = [] {
    "FLL: frequency estimation on a clean tone"_test = [] {
        gr::digital::FllBandEdgeCF fll;
        const float omega = 0.20f;      // radians/sample
        fll.start(/*sps*/4.0f, /*rolloff*/0.35f, /*ntaps*/45, /*loop_bw*/0.01f);

        const auto in = tone(6000, omega);
        std::vector<std::complex<float>> out;
        out.reserve(in.size());

        std::complex<float> y{};
        for (const auto& x : in) {
            fll.processOne(x, y);
            out.push_back(y);
        }

        const std::size_t N0 = 1000;
        float f_acc = 0.0f;
        std::size_t cnt = 0;
        for (std::size_t i = N0; i < in.size(); ++i) {
            f_acc += fll.last_freq();
            ++cnt;
        }
        const float f_avg = f_acc / static_cast<float>(cnt);
        expect(std::abs(f_avg - omega) < 0.01f);

        std::size_t stable = 0;
        for (std::size_t i = N0 + 1; i < out.size(); ++i) {
            const auto z = out[i] * std::conj(out[i-1]);
            const float dphi = std::atan2(z.imag(), z.real());
            if (std::abs(dphi) < 0.05f) ++stable;
        }
        expect(stable >= (out.size() - (N0 + 1)) * 9 / 10);
        fll.stop();
    };

    "FLL: small noise, frequency still accurate"_test = [] {
        gr::digital::FllBandEdgeCF fll;
        const float omega = -0.15f; // negative frequency
        fll.start(4.0f, 0.35f, 45, 0.02f);

        std::vector<std::complex<float>> in;
        in.reserve(8000);
        float ph = 0.0f;
        std::mt19937 rng(99);
        std::normal_distribution<float> N(0.0f, 0.03f);
        for (std::size_t i = 0; i < 8000; ++i) {
            const auto s = std::complex<float>(std::cos(ph), std::sin(ph));
            in.emplace_back(s.real() + N(rng), s.imag() + N(rng));
            ph += omega;
            if (ph >  kPi) ph -= 2.0f * kPi;
            if (ph < -kPi) ph += 2.0f * kPi;
        }

        std::complex<float> y{};
        for (const auto& x : in) fll.processOne(x, y);

        const std::size_t N0 = 1500;
        float f_acc = 0.0f;
        std::size_t cnt = 0;
        for (std::size_t i = N0; i < in.size(); ++i) {
            f_acc += fll.last_freq();
            ++cnt;
        }
        const float f_avg = f_acc / static_cast<float>(cnt);
        expect(std::abs(f_avg - omega) < 0.02f);
        fll.stop();
    };
};
