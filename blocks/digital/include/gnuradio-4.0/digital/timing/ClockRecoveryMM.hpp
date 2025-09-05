#ifndef GNURADIO_DIGITAL_TIMING_CLOCKRECOVERYMM_HPP
#define GNURADIO_DIGITAL_TIMING_CLOCKRECOVERYMM_HPP

#include <complex>
#include <algorithm>
#include <type_traits>
#include <cstddef>
#include <cmath>

namespace gr::digital {

namespace detail {
inline float slicer(float v) { return v >= 0.0f ? 1.0f : -1.0f; }
inline std::complex<float> slicer(const std::complex<float>& v) {
    return { slicer(v.real()), slicer(v.imag()) };
}
} // namespace detail

template <typename T>
class ClockRecoveryMM {
public:
    ClockRecoveryMM() = default;

    void start(float omega, float gain_omega, float mu, float gain_mu, float omega_relative_limit)
    {
        omega0_ = std::max(1e-6f, omega);
        omega_  = omega0_;
        // interpret mu in [0,1) as fractional phase of a symbol; store in [0, omega)
        mu_ = std::clamp(mu, 0.0f, 1.0f) * omega_;
        g_omega_ = std::max(0.0f, gain_omega);
        g_mu_    = std::max(0.0f, gain_mu);
        rel_lim_ = std::max(0.0f, omega_relative_limit);
        last_    = T{};
        started_ = true;
    }

    void stop() { started_ = false; }

    bool processOne(const T& x, T& y)
    {
        if (!started_) return false;

        mu_ += 1.0f;                 
        bool emit = false;
        if (mu_ >= omega_) {         
            mu_ -= omega_;
            y = detail::slicer(x);   
            emit = true;
        }

        last_ = x;
        return emit;
    }

private:
    static float timing_error_(const float& x, const float& prev) {
        return detail::slicer(prev) * (x - prev);
    }
    static float timing_error_(const std::complex<float>& x,
                               const std::complex<float>& prev) {
        const auto si = detail::slicer(prev.real());
        const auto sq = detail::slicer(prev.imag());
        return si * (x.real() - prev.real()) + sq * (x.imag() - prev.imag());
    }

    float omega0_{2.0f};   
    float omega_{2.0f};    
    float mu_{0.0f};       
    float g_omega_{0.0f};
    float g_mu_{0.0f};
    float rel_lim_{0.0f};
    T     last_{};
    bool  started_{false};
};

using ClockRecoveryMMf  = ClockRecoveryMM<float>;
using ClockRecoveryMMcf = ClockRecoveryMM<std::complex<float>>;

} // namespace gr::digital

#endif // GNURADIO_DIGITAL_TIMING_CLOCKRECOVERYMM_HPP
