#ifndef GNURADIO_DIGITAL_EQUALIZER_ADAPTIVEALGORITHM_HPP
#define GNURADIO_DIGITAL_EQUALIZER_ADAPTIVEALGORITHM_HPP

#include <vector>
#include <complex>
#include <cstddef>
#include <algorithm>
#include <cmath>

namespace gr::digital {

enum class AdaptAlg { LMS, NLMS, CMA };

struct AdaptiveEQCF {
    using cfloat = std::complex<float>;

    AdaptAlg alg{AdaptAlg::LMS};
    float    mu{0.01f};
    float    cma_R{1.0f};
    float    nlms_eps{1e-6f};

    std::vector<cfloat> w;   
    std::vector<cfloat> dl;  
    std::size_t L{0};

    void start(std::size_t num_taps,
               float step_mu,
               AdaptAlg which = AdaptAlg::LMS,
               float cma_modulus = 1.0f)
    {
        L = std::max<std::size_t>(1, num_taps);
        alg = which;
        mu  = step_mu;
        cma_R = cma_modulus;

        w.assign(L, cfloat{0.f,0.f});
        w[0] = cfloat{1.f, 0.f};           
        dl.assign(L, cfloat{0.f,0.f});
    }

    void stop() { w.clear(); dl.clear(); L = 0; }

    bool processOneTrain(const cfloat& x, const cfloat& d, cfloat& y_out) {
        if (L == 0) return false;
        pushfront_(x);
        const cfloat y = dot_wHx_();
        y_out = y;
        const cfloat e = d - y;
        lms_like_update_(e);
        return true;
    }

    bool processOneDD(const cfloat& x, cfloat& y_out) {
        if (L == 0) return false;
        pushfront_(x);
        const cfloat y = dot_wHx_();
        y_out = y;

        if (alg == AdaptAlg::CMA) {
            const float  y2 = std::norm(y);
            const cfloat e_cma = y * (y2 - cma_R);
            cma_update_(e_cma);
        } else {
            const cfloat d = slicer_bpsk_I_(y);   // *** BPSK-on-I slicer ***
            const cfloat e = d - y;
            lms_like_update_(e);
        }
        return true;
    }

private:
    inline void pushfront_(const cfloat& x) noexcept {
        for (std::size_t i = L-1; i > 0; --i) dl[i] = dl[i-1];
        dl[0] = x;
    }

    // y = w^H x
    inline cfloat dot_wHx_() const noexcept {
        cfloat acc{0.f,0.f};
        for (std::size_t i = 0; i < L; ++i)
            acc += std::conj(w[i]) * dl[i];
        return acc;
    }

    inline void lms_like_update_(const cfloat& e) noexcept {
        if (alg == AdaptAlg::NLMS) {
            float p = nlms_eps;
            for (std::size_t i = 0; i < L; ++i) p += std::norm(dl[i]);
            const float mu_n = mu / p;
            const cfloat ce = std::conj(e);
            for (std::size_t i = 0; i < L; ++i)
                w[i] += mu_n * dl[i] * ce;
        } else { // LMS
            const cfloat ce = std::conj(e);
            for (std::size_t i = 0; i < L; ++i)
                w[i] += mu * dl[i] * ce;
        }
    }

    inline void cma_update_(const cfloat& e_cma) noexcept {
        const cfloat ce = std::conj(e_cma);
        for (std::size_t i = 0; i < L; ++i)
            w[i] -= mu * dl[i] * ce;
    }

    static inline cfloat slicer_bpsk_I_(const cfloat& y) noexcept {
        const float r = (y.real() >= 0.f) ? 1.f : -1.f;
        return {r, 0.f};
    }
};

} // namespace gr::digital

#endif // GNURADIO_DIGITAL_EQUALIZER_ADAPTIVEALGORITHM_HPP
