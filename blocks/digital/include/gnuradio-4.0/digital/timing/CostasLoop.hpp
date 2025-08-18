#ifndef GNURADIO_DIGITAL_TIMING_COSTASLOOP_HPP
#define GNURADIO_DIGITAL_TIMING_COSTASLOOP_HPP

#include <array>
#include <cmath>
#include <complex>
#include <cstdint>

namespace gr::digital {

struct CostasLoopCF {
    using cfloat = std::complex<float>;
    static constexpr float kPi = 3.14159265358979323846f;

    float phase_{0.0f};
    float freq_{0.0f};
    float alpha_{0.0f};
    float beta_{0.0f};
    unsigned order_{2}; // 2,4,8 supported

    void start(float loop_bw, unsigned order) {
        order_ = order ? order : 2u;
        // classic 2nd-order loop gains from normalized bandwidth
        const float zeta = 0.7071f;
        const float BL   = loop_bw;
        const float den  = 1.0f + 2.0f * zeta * BL + BL * BL;
        alpha_ = (BL > 0.0f) ? (4.0f * zeta * BL) / den : 0.0f;
        beta_  = (BL > 0.0f) ? (4.0f * BL * BL) / den : 0.0f;

        phase_ = 0.0f;
        freq_  = 0.0f;
    }

    void stop() {}

    bool processOne(const cfloat& x, cfloat& y) {
        const float c = std::cosf(-phase_);
        const float s = std::sinf(-phase_);
        const cfloat nco{c, s};
        const cfloat v = x * nco; // basebanded sample
        y = v;                    // output

        float e = 0.0f;

        if (order_ == 2u) {
            const float sgnI = (v.real() >= 0.0f) ? 1.0f : -1.0f;
            e = sgnI * v.imag();

        } else if (order_ == 4u) {
            const float ur = (v.real() >= 0.0f) ? (0.70710678f) : (-0.70710678f);
            const float ui = (v.imag() >= 0.0f) ? (0.70710678f) : (-0.70710678f);
            e = v.imag() * ur - v.real() * ui;

        } else if (order_ == 8u) {
            const float theta = std::atan2f(v.imag(), v.real());
            const int   k     = static_cast<int>(std::lroundf(8.0f * theta / (2.0f * kPi)));
            const float ref   = (2.0f * kPi / 8.0f) * static_cast<float>(k);
            const float ur    = std::cosf(ref);
            const float ui    = std::sinf(ref);
            e = v.imag() * ur - v.real() * ui;

        } else {
            const float M     = static_cast<float>(order_);
            const float theta = std::atan2f(v.imag(), v.real());
            const int   k     = static_cast<int>(std::lroundf(M * theta / (2.0f * kPi)));
            const float ref   = (2.0f * kPi / M) * static_cast<float>(k);
            const float ur    = std::cosf(ref);
            const float ui    = std::sinf(ref);
            e = v.imag() * ur - v.real() * ui;
        }

        freq_  += beta_ * e;
        phase_ += freq_ + alpha_ * e;

        if (phase_ >  kPi) phase_ -= 2.0f * kPi;
        if (phase_ < -kPi) phase_ += 2.0f * kPi;

        return true;
    }
};

} // namespace gr::digital

#endif // GNURADIO_DIGITAL_TIMING_COSTASLOOP_HPP
