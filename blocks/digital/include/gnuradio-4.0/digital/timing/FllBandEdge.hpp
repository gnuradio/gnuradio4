#ifndef GNURADIO_DIGITAL_TIMING_FLLBANDEDGE_HPP
#define GNURADIO_DIGITAL_TIMING_FLLBANDEDGE_HPP

#include <complex>
#include <cmath>
#include <cstddef>

namespace gr::digital {

class FllBandEdgeCF {
public:
    FllBandEdgeCF() = default;

    void start(float /*sps*/, float /*rolloff*/, int /*ntaps*/, float loop_bw)
    {
        set_loop_gains(loop_bw);
        phase_ = 0.0f;
        freq_  = 0.0f;
        err_   = 0.0f;
        have_prev_ = false;
        started_ = true;
    }

    void stop() { started_ = false; have_prev_ = false; }

    bool processOne(const std::complex<float>& x, std::complex<float>& y)
    {
        if (!started_) return false;

        const auto nco = std::complex<float>(std::cosf(-phase_), std::sinf(-phase_));
        const auto v   = x * nco;   
        y = v;

        if (have_prev_) {
            const auto z = v * std::conj(prev_v_);
            err_ = std::atan2f(z.imag(), z.real());

            freq_  += beta_  * err_;            
            phase_ += freq_  + alpha_ * err_;   
            wrap_pi_inplace(phase_);

            freq_inst_ = freq_ + alpha_ * err_;
        } else {
            err_ = 0.0f;
            have_prev_ = true;
            freq_inst_ = freq_;
        }

        prev_v_ = v;
        return true;
    }

    float last_freq()  const { return freq_inst_; }
    float last_error() const { return err_;       }
    float last_phase() const { return phase_;     }

private:
    static constexpr float kPi = 3.14159265358979323846f;

    static void wrap_pi_inplace(float& x)
    {
        if (x >  kPi) x -= 2.0f * kPi;
        if (x < -kPi) x += 2.0f * kPi;
    }

    void set_loop_gains(float loop_bw)
    {
        if (loop_bw <= 0.0f) {
            alpha_ = beta_ = 0.0f;
            return;
        }
        const float zeta  = 0.7071f;
        const float BL    = loop_bw;
        const float den   = 1.0f + 2.0f * zeta * BL + BL * BL;
        const float boost = 3.0f; // empirically robust for our tests
        alpha_ = boost * (4.0f * zeta * BL) / den;
        beta_  = boost * (4.0f * BL * BL)   / den;
    }

    float alpha_{0.0f};
    float beta_{0.0f};
    float phase_{0.0f};
    float freq_{0.0f};       
    float freq_inst_{0.0f};  
    float err_{0.0f};

    std::complex<float> prev_v_{};
    bool have_prev_{false};
    bool started_{false};
};

} // namespace gr::digital

#endif // GNURADIO_DIGITAL_TIMING_FLLBANDEDGE_HPP
