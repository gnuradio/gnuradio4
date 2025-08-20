#ifndef GNURADIO_DIGITAL_EQUALIZER_DECISIONFEEDBACKEQUALIZER_HPP
#define GNURADIO_DIGITAL_EQUALIZER_DECISIONFEEDBACKEQUALIZER_HPP

#include <vector>
#include <complex>
#include <cstddef>
#include <algorithm>
#include <cmath>

namespace gr::digital {

enum class DfeAlg { LMS, NLMS, CMA };

struct DecisionFeedbackEqualizerCF {
    using cfloat = std::complex<float>;

    std::size_t Lf{7};
    std::size_t Lb{3};
    unsigned    sps{1};
    DfeAlg      alg{DfeAlg::LMS};
    float       mu_f{0.01f};
    float       mu_b{0.01f};
    float       cma_R{1.0f};
    float       nlms_eps{1e-6f};
    bool        adapt_after_training{true};
    std::vector<cfloat> training;

    std::vector<cfloat> wf, wb;
    std::vector<cfloat> xdl, ddl;
    unsigned            phase{0};
    bool                started{false};

    void start(std::size_t num_taps_forward,
               std::size_t num_taps_feedback,
               unsigned sps_in,
               DfeAlg which,
               float mu_forward,
               float mu_feedback,
               float cma_modulus,
               bool adapt_after,
               const std::vector<cfloat>& training_seq)
    {
        Lf = std::max<std::size_t>(1, num_taps_forward);
        Lb = std::max<std::size_t>(0, num_taps_feedback);
        sps = std::max<unsigned>(1u, sps_in);
        alg = which;
        mu_f = mu_forward;
        mu_b = mu_feedback;
        cma_R = cma_modulus;
        adapt_after_training = adapt_after;
        training = training_seq;

        wf.assign(Lf, cfloat{0.f,0.f}); wf[0] = cfloat{1.f,0.f};
        wb.assign(Lb, cfloat{0.f,0.f});
        xdl.assign(Lf, cfloat{0.f,0.f});
        ddl.assign(Lb, cfloat{0.f,0.f});

        phase = 0;
        started = true;
    }

    void stop() {
        wf.clear(); wb.clear(); xdl.clear(); ddl.clear();
        started = false; phase = 0;
    }

    std::size_t equalize(const cfloat* in,
                         cfloat* out,
                         unsigned num_inputs,
                         unsigned max_num_outputs,
                         const std::vector<unsigned>& training_start_samples = {},
                         bool /*history_included*/ = false,
                         std::vector<std::vector<cfloat>>* taps_out = nullptr,
                         std::vector<unsigned short>* state_out = nullptr)
    {
        if (!started || !in || !out || max_num_outputs == 0) return 0;

        int t_start = -1;
        if (!training.empty()) {
            for (auto s : training_start_samples) {
                if (s < num_inputs) { t_start = static_cast<int>(s); break; }
            }
        }

        std::size_t out_count = 0;
        std::size_t t_pos = 0;
        bool in_training = (t_start == 0);

        for (unsigned n = 0; n < num_inputs; ++n) {
            for (std::size_t i = Lf - 1; i > 0; --i) xdl[i] = xdl[i - 1];
            xdl[0] = in[n];

            if (!training.empty() && t_start >= 0 && static_cast<int>(n) == t_start) {
                in_training = true;
                t_pos = 0;
            }

            if (phase + 1 == sps) {
                // y = wf^H x - wb^H d_hist
                const cfloat y = dot_wHx_(wf, xdl) - dot_wHx_(wb, ddl);

                cfloat e{};
                if (in_training && t_pos < training.size()) {
                    e = training[t_pos] - y;
                } else if (alg == DfeAlg::CMA) {
                    const float y2 = std::norm(y);
                    e = y * (y2 - cma_R);
                } else {
                    const cfloat d_hat = slicer_bpsk_I_(y);
                    e = d_hat - y;
                }

                if (in_training || alg != DfeAlg::CMA) {
                    if (alg == DfeAlg::NLMS) {
                        float pf = nlms_eps, pb = nlms_eps;
                        for (std::size_t i = 0; i < Lf; ++i) pf += std::norm(xdl[i]);
                        for (std::size_t i = 0; i < Lb; ++i) pb += std::norm(ddl[i]);
                        const float muf = mu_f / pf;
                        const float mub = mu_b / pb;
                        const cfloat ce = std::conj(e);
                        for (std::size_t i = 0; i < Lf; ++i) wf[i] += muf * xdl[i] * ce;
                        for (std::size_t i = 0; i < Lb; ++i) wb[i] -= mub * ddl[i] * ce;
                    } else { // LMS
                        const cfloat ce = std::conj(e);
                        for (std::size_t i = 0; i < Lf; ++i) wf[i] += mu_f * xdl[i] * ce;
                        for (std::size_t i = 0; i < Lb; ++i) wb[i] -= mu_b * ddl[i] * ce;
                    }
                } else {
                    const cfloat ce_cma = std::conj(e);
                    for (std::size_t i = 0; i < Lf; ++i) wf[i] -= mu_f * xdl[i] * ce_cma;
                    const cfloat dres = slicer_bpsk_I_(y) - y;
                    const cfloat ce_d = std::conj(dres);
                    for (std::size_t i = 0; i < Lb; ++i) wb[i] -= mu_b * ddl[i] * ce_d;
                }

                if (out_count < max_num_outputs) out[out_count++] = y;
                if (taps_out) {
                    std::vector<cfloat> pack; pack.reserve(Lf + Lb);
                    pack.insert(pack.end(), wf.begin(), wf.end());
                    pack.insert(pack.end(), wb.begin(), wb.end());
                    taps_out->push_back(std::move(pack));
                }
                if (state_out) state_out->push_back(in_training ? 1u : 2u);

                const cfloat dcur = (in_training && t_pos < training.size())
                                  ? training[t_pos]
                                  : slicer_bpsk_I_(y);
                if (Lb > 0) {
                    for (std::size_t i = Lb - 1; i > 0; --i) ddl[i] = ddl[i - 1];
                    ddl[0] = dcur;
                }

                if (in_training && t_pos < training.size()) {
                    ++t_pos;
                    if (t_pos >= training.size()) {
                        in_training = false;
                        if (!adapt_after_training) { mu_f = 0.0f; mu_b = 0.0f; }
                    }
                }
                phase = 0;
            } else {
                if (state_out) state_out->push_back(0u);
                ++phase;
            }
        }
        return out_count;
    }

private:
    static inline cfloat slicer_bpsk_I_(const cfloat& y) noexcept {
        const float r = (y.real() >= 0.f) ? 1.f : -1.f;
        return {r, 0.f};
    }

    static inline cfloat dot_wHx_(const std::vector<cfloat>& w, const std::vector<cfloat>& x) noexcept {
        cfloat acc{0.f,0.f};
        const std::size_t L = std::min(w.size(), x.size());
        for (std::size_t i = 0; i < L; ++i) acc += std::conj(w[i]) * x[i];
        return acc;
    }
};

} // namespace gr::digital

#endif // GNURADIO_DIGITAL_EQUALIZER_DECISIONFEEDBACKEQUALIZER_HPP
