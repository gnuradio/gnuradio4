#ifndef GNURADIO_DIGITAL_OFDM_CYCLICPREFIXER_HPP
#define GNURADIO_DIGITAL_OFDM_CYCLICPREFIXER_HPP

#include <vector>
#include <complex>
#include <cstddef>
#include <stdexcept>
#include <algorithm>

namespace gr::digital {

struct OfdmCyclicPrefixerCF {
    using cfloat = std::complex<float>;

    std::size_t fft_len{0};
    std::vector<int> cp_lengths;
    int uniform_cp{-1};
    int rolloff_len{0};   // modeled: 0 or 2
    bool framed{false};

    std::size_t cp_index{0};
    float prev_tail{0.0f};        
    std::vector<cfloat> last_symbol;

    void start(std::size_t N, int cp_len, int rolloff = 0, bool framed_mode = false)
    {
        if (N == 0) throw std::invalid_argument("fft_len must be > 0");
        if (cp_len < 0) throw std::invalid_argument("cp_len must be >= 0");
        fft_len = N;
        uniform_cp = cp_len;
        cp_lengths.clear();
        rolloff_len = rolloff;
        framed = framed_mode;
        cp_index = 0;
        prev_tail = 0.0f;
        last_symbol.clear();
    }

    void start(std::size_t N, const std::vector<int>& cp_vec, int rolloff = 0, bool framed_mode = false)
    {
        if (N == 0) throw std::invalid_argument("fft_len must be > 0");
        if (cp_vec.empty()) throw std::invalid_argument("cp_lengths must not be empty");
        for (int c : cp_vec) if (c < 0) throw std::invalid_argument("cp length < 0 not allowed");
        fft_len = N;
        cp_lengths = cp_vec;
        uniform_cp = -1;
        rolloff_len = rolloff;
        framed = framed_mode;
        cp_index = 0;
        prev_tail = 0.0f;
        last_symbol.clear();
    }

    void stop() {
        fft_len = 0;
        cp_lengths.clear();
        uniform_cp = -1;
        rolloff_len = 0;
        framed = false;
        cp_index = 0;
        prev_tail = 0.0f;
        last_symbol.clear();
    }

    void processOne(const cfloat* symbol, std::vector<cfloat>& out)
    {
        if (fft_len == 0 || symbol == nullptr) return;

        int cp_i = (uniform_cp >= 0) ? uniform_cp
                                     : cp_lengths[cp_index % cp_lengths.size()];
        ++cp_index;

        const std::size_t cp = static_cast<std::size_t>(cp_i);
        const std::size_t base = (cp <= fft_len) ? (fft_len - cp) : 0;

        if (cp > 0) {
            if (rolloff_len == 2) {
                const cfloat cp0 = symbol[base];
                const float y0r = prev_tail + 0.5f * cp0.real();
                out.emplace_back(y0r, 0.0f);
                for (std::size_t k = 1; k < cp; ++k) {
                    out.push_back(symbol[base + k]);
                }
            } else {
                for (std::size_t k = 0; k < cp; ++k) {
                    out.push_back(symbol[base + k]);
                }
            }
        }

        for (std::size_t n = 0; n < fft_len; ++n) {
            out.push_back(symbol[n]);
        }

        if (rolloff_len == 2) {
            prev_tail = 0.5f * symbol[0].real();
        } else {
            prev_tail = 0.0f;
        }
        last_symbol.assign(symbol, symbol + std::min<std::size_t>(1, fft_len));
    }

    void finalize(std::vector<cfloat>& out)
    {
        if (framed && rolloff_len == 2 && !last_symbol.empty()) {
            const float tail = 0.5f * last_symbol[0].real();
            out.emplace_back(tail, 0.0f);
        }
    }
};

} // namespace gr::digital

#endif // GNURADIO_DIGITAL_OFDM_CYCLICPREFIXER_HPP
