#ifndef GNURADIO_DIGITAL_MEASURE_MPSKSNREST_HPP
#define GNURADIO_DIGITAL_MEASURE_MPSKSNREST_HPP

#include <complex>
#include <cmath>
#include <cstddef>
#include <algorithm>

namespace gr::digital {

struct MpskSnrBase {
    static inline void ewma(double& acc, double x, double alpha) noexcept {
        acc = (1.0 - alpha) * acc + alpha * x;
    }
    static inline double to_db(double x) noexcept {
        return 10.0 * std::log10(std::max(x, 1e-30));
    }
};

struct MpskSnrM2M4 : MpskSnrBase {
    double alpha{0.001};
    double m2{0.0};
    double m4{0.0};

    void start(double a = 0.001) { alpha = std::clamp(a, 1e-6, 1.0); m2 = 0.0; m4 = 0.0; }
    void stop() {}

    inline void processOne(const std::complex<float>& x) noexcept {
        const double p2 = static_cast<double>(std::norm(x));
        const double p4 = p2 * p2;
        ewma(m2, p2, alpha);
        ewma(m4, p4, alpha);
    }

    inline double snr_linear() const noexcept {
        const double P  = std::max(m2, 0.0);
        const double twoP2_minus_R = std::max(0.0, 2.0 * P * P - m4);
        const double Ps = std::sqrt(twoP2_minus_R);
        const double Pn = std::max(P - Ps, 1e-30);
        return Ps / Pn;
    }
    inline double snr_db() const noexcept { return to_db(snr_linear()); }
};

struct MpskSnrSimple : MpskSnrBase {
    enum class SimpleMode { BPSK_I, QPSK };

    double alpha{0.001};
    double Ptot{0.0};
    double Pres{0.0};
    SimpleMode mode{SimpleMode::BPSK_I};

    void start(double a = 0.001, SimpleMode m = SimpleMode::BPSK_I) {
        alpha = std::clamp(a, 1e-6, 1.0);
        Ptot = 0.0; Pres = 0.0; mode = m;
    }
    void stop() {}

    static inline std::complex<float> slicer_qpsk(const std::complex<float>& x) noexcept {
        const float r = (x.real() >= 0.0f) ? 1.0f : -1.0f;
        const float i = (x.imag() >= 0.0f) ? 1.0f : -1.0f;
        return {r, i};
    }
    static inline std::complex<float> slicer_bpsk_i(const std::complex<float>& x) noexcept {
        const float r = (x.real() >= 0.0f) ? 1.0f : -1.0f;
        return {r, 0.0f};
    }

    inline void processOne(const std::complex<float>& x) noexcept {
        const double p  = static_cast<double>(std::norm(x));
        const auto shat = (mode == SimpleMode::BPSK_I) ? slicer_bpsk_i(x) : slicer_qpsk(x);
        const double e  = static_cast<double>(std::norm(x - shat));
        ewma(Ptot, p,  alpha);
        ewma(Pres, e,  alpha * 0.5); // slightly slower residual smoothing
    }

    inline double snr_linear() const noexcept {
        const double Psig = std::max(Ptot - Pres, 1e-30);
        const double Pn   = std::max(Pres,      1e-30);
        return Psig / Pn;
    }
    inline double snr_db() const noexcept { return to_db(snr_linear()); }
};

} // namespace gr::digital

#endif // GNURADIO_DIGITAL_MEASURE_MPSKSNREST_HPP



