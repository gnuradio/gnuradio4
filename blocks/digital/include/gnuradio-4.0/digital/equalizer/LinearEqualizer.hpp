#ifndef GNURADIO_DIGITAL_EQUALIZER_LINEAREQUALIZER_HPP
#define GNURADIO_DIGITAL_EQUALIZER_LINEAREQUALIZER_HPP

#include <gnuradio-4.0/digital/equalizer/AdaptiveAlgorithm.hpp>

#include <vector>
#include <complex>
#include <cstddef>
#include <algorithm>

namespace gr::digital {

struct LinearEqualizerCF {
    using cfloat = std::complex<float>;

    std::size_t  L{7};
    unsigned     sps{1};
    AdaptAlg     alg{AdaptAlg::LMS};
    float        mu{0.01f};
    float        cma_R{1.0f};
    bool         adapt_after_training{true};
    std::vector<cfloat> training;

    AdaptiveEQCF eq;
    unsigned     phase{0};
    bool         started{false};

    void start(std::size_t num_taps,
               unsigned sps_in,
               AdaptAlg which,
               float step_mu,
               float cma_modulus,
               bool adapt_after,
               const std::vector<cfloat>& training_seq)
    {
        L = std::max<std::size_t>(1, num_taps);
        sps = std::max<unsigned>(1u, sps_in);
        alg = which;
        mu  = step_mu;
        cma_R = cma_modulus;
        adapt_after_training = adapt_after;
        training = training_seq;

        eq.start(L, mu, alg, cma_R);
        phase = 0;
        started = true;
    }

    void stop() {
        eq.stop();
        started = false;
        phase = 0;
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
            if (!training.empty() && t_start >= 0 && static_cast<int>(n) == t_start) {
                in_training = true;
                t_pos = 0;
            }

            if (phase + 1 == sps) {
                if (in_training && t_pos < training.size()) {
                    cfloat y{};
                    eq.processOneTrain(in[n], training[t_pos], y);
                    ++t_pos;

                    if (taps_out) taps_out->push_back(eq.w);
                    if (state_out) state_out->push_back(static_cast<unsigned short>(1)); 

                    if (out_count < max_num_outputs) out[out_count++] = y;

                    if (t_pos >= training.size()) {
                        in_training = false;
                        if (!adapt_after_training) {
                            eq.mu = 0.0f; 
                        }
                    }
                } else {
                    cfloat y{};
                    eq.processOneDD(in[n], y);
                    if (taps_out) taps_out->push_back(eq.w);
                    if (state_out) state_out->push_back(static_cast<unsigned short>(2)); // DD
                    if (out_count < max_num_outputs) out[out_count++] = y;
                }
                phase = 0;
            } else {
                cfloat ytmp{};
                eq.processOneDD(in[n], ytmp);
                if (state_out) state_out->push_back(static_cast<unsigned short>(0)); 
                ++phase;
            }
        }
        return out_count;
    }
};

} // namespace gr::digital

#endif // GNURADIO_DIGITAL_EQUALIZER_LINEAREQUALIZER_HPP
