#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/timing/CostasLoop.hpp>

#include <vector>
#include <complex>
#include <random>
#include <cmath>
#include <cstddef>

using namespace boost::ut;

namespace {

constexpr float kPi = 3.14159265358979323846f;

static std::vector<std::complex<float>> make_bpsk_rot(std::size_t n, float rot_rad)
{
    std::mt19937 rng(123);
    std::uniform_int_distribution<int> bit(0,1);
    std::vector<std::complex<float>> v;
    v.reserve(n);
    const auto r = std::complex<float>(std::cos(rot_rad), std::sin(rot_rad));
    for (std::size_t i = 0; i < n; ++i) {
        const float s = bit(rng) ? +1.0f : -1.0f;
        v.emplace_back(r * std::complex<float>(s, 0.0f));
    }
    return v;
}

static std::vector<std::complex<float>> make_qpsk_rot(std::size_t n, float rot_rad)
{
    std::mt19937 rng(321);
    std::uniform_int_distribution<int> b(0,1);
    std::vector<std::complex<float>> v;
    v.reserve(n);
    const auto r = std::complex<float>(std::cos(rot_rad), std::sin(rot_rad));
    for (std::size_t i = 0; i < n; ++i) {
        const float I = b(rng) ? +1.0f : -1.0f;
        const float Q = b(rng) ? +1.0f : -1.0f;
        v.emplace_back(r * std::complex<float>(I, Q));
    }
    return v;
}

static std::vector<std::complex<float>> make_8psk_rot(std::size_t n, float rot_rad)
{
    std::mt19937 rng(777);
    std::uniform_int_distribution<int> k(0,7);
    std::vector<std::complex<float>> v;
    v.reserve(n);
    const auto r = std::complex<float>(std::cos(rot_rad), std::sin(rot_rad));
    for (std::size_t i = 0; i < n; ++i) {
        const float a = 2.0f * kPi * static_cast<float>(k(rng)) / 8.0f;
        const auto p = std::complex<float>(std::cos(a), std::sin(a));
        v.emplace_back(r * p);
    }
    return v;
}

inline bool close(const std::complex<float>& a, const std::complex<float>& b, float tol)
{
    return std::abs(a.real() - b.real()) <= tol && std::abs(a.imag() - b.imag()) <= tol;
}

} // namespace

const suite CostasLoopSuite = [] {
    "CostasLoop: pass-through when loop_bw=0"_test = [] {
        gr::digital::CostasLoopCF loop;
        loop.start(0.0f, 2);

        std::vector<std::complex<float>> in(100, {1.0f, 0.0f});
        std::vector<std::complex<float>> out;
        out.reserve(in.size());
        std::complex<float> y{};
        for (const auto& x : in) {
            if (loop.processOne(x, y)) out.push_back(y);
        }

        expect(out.size() == in.size());
        for (std::size_t i = 0; i < out.size(); ++i)
            expect(close(out[i], in[i], 1e-6f));
        loop.stop();
    };

    "CostasLoop: BPSK convergence from small rotation"_test = [] {
        gr::digital::CostasLoopCF loop;
        loop.start(0.25f, 2);

        const float rot = 0.2f; // radians
        const auto in   = make_bpsk_rot(200, rot);
        std::vector<std::complex<float>> out;
        out.reserve(in.size());
        std::complex<float> y{};
        for (const auto& x : in) loop.processOne(x, y), out.push_back(y);

        const std::size_t N0 = 60;
        std::size_t ok = 0;
        for (std::size_t i = N0; i < out.size(); ++i) {
            const float I = (out[i].real() >= 0.0f) ? +1.0f : -1.0f;
            if (std::abs(out[i].real() - I) < 0.1f && std::abs(out[i].imag()) < 0.1f) ++ok;
        }
        expect(ok >= (out.size() - N0) * 9 / 10);
        loop.stop();
    };

    "CostasLoop: QPSK convergence from small rotation"_test = [] {
        gr::digital::CostasLoopCF loop;
        loop.start(0.25f, 4);

        const float rot = 0.2f;
        const auto in   = make_qpsk_rot(200, rot);
        std::vector<std::complex<float>> out;
        out.reserve(in.size());
        std::complex<float> y{};
        for (const auto& x : in) loop.processOne(x, y), out.push_back(y);

        const std::size_t N0 = 60;
        std::size_t ok = 0;
        for (std::size_t i = N0; i < out.size(); ++i) {
            const float I = (out[i].real() >= 0.0f) ? +1.0f : -1.0f;
            const float Q = (out[i].imag() >= 0.0f) ? +1.0f : -1.0f;
            if (std::abs(out[i].real() - I) < 0.2f && std::abs(out[i].imag() - Q) < 0.2f) ++ok;
        }
        expect(ok >= (out.size() - N0) * 8 / 10);
        loop.stop();
    };

    "CostasLoop: 8PSK convergence from small rotation"_test = [] {
        gr::digital::CostasLoopCF loop;
        loop.start(0.2f, 8);

        const float rot = 0.1f;
        const auto in   = make_8psk_rot(240, rot);
        std::vector<std::complex<float>> out;
        out.reserve(in.size());
        std::complex<float> y{};
        for (const auto& x : in) loop.processOne(x, y), out.push_back(y);

        const std::size_t N0 = 80;
        std::size_t ok = 0;
        for (std::size_t i = N0; i < out.size(); ++i) {
            const float mag = std::abs(out[i]);
            const float ang = std::atan2(out[i].imag(), out[i].real());
            const float k   = std::round(ang * 8.0f / (2.0f * kPi));
            const float ref = k * (2.0f * kPi / 8.0f);
            if (std::abs(mag - 1.0f) < 0.2f && std::abs(ang - ref) < 0.25f) ++ok;
        }
        expect(ok >= (out.size() - N0) * 7 / 10);
        loop.stop();
    };
};
