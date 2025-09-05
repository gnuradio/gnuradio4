#ifndef GNURADIO_DIGITAL_OFDM_SERIALIZER_HPP
#define GNURADIO_DIGITAL_OFDM_SERIALIZER_HPP

#include <vector>
#include <complex>
#include <cstddef>
#include <stdexcept>

namespace gr::digital {

struct OfdmSerializerVCC {
    using cfloat = std::complex<float>;

    std::size_t fft_len{0};
    std::vector<std::vector<int>> occ;  
    bool input_is_shifted{true};
    int  carrier_offset{0};             

    void start(std::size_t N,
               const std::vector<std::vector<int>>& occupied_carriers,
               bool input_shifted = true)
    {
        if (N == 0) throw std::invalid_argument("fft_len must be > 0");
        if (occupied_carriers.empty()) throw std::invalid_argument("occupied_carriers empty");
        fft_len = N;
        occ = occupied_carriers;
        input_is_shifted = input_shifted;

        for (const auto& v : occ) {
            if (v.empty()) throw std::invalid_argument("occupied_carriers contains empty set");
        }
    }

    void stop() { occ.clear(); fft_len = 0; carrier_offset = 0; }

    void set_carrier_offset(int offset) { carrier_offset = offset; }

    void processSymbols(const cfloat* time_bins,
                        std::size_t n_syms,
                        std::vector<cfloat>& out) const
    {
        if (fft_len == 0) return;
        if (!time_bins) throw std::invalid_argument("null input");

        const std::size_t sets = occ.size();
        const int N  = static_cast<int>(fft_len);
        const int Nh = N / 2; // assumes even N

        for (std::size_t s = 0; s < n_syms; ++s) {
            const auto& mask = occ[s % sets];
            const cfloat* sym = time_bins + s * fft_len;

            for (int b_raw : mask) {
                const int b_eff = b_raw + carrier_offset;

                int pos = 0;
                if (input_is_shifted) {
                    int t = b_eff + Nh;
                    t %= N; if (t < 0) t += N;
                    pos = t;
                } else {
                    int t = b_eff % N; if (t < 0) t += N;
                    pos = t;
                }

                out.push_back(sym[static_cast<std::size_t>(pos)]);
            }
        }
    }
};

} // namespace gr::digital

#endif // GNURADIO_DIGITAL_OFDM_SERIALIZER_HPP
