#ifndef GNURADIO_DIGITAL_OFDM_CARRIERALLOCATOR_HPP
#define GNURADIO_DIGITAL_OFDM_CARRIERALLOCATOR_HPP

#include <vector>
#include <complex>
#include <cstddef>
#include <stdexcept>
#include <algorithm>

namespace gr::digital {

struct OfdmCarrierAllocatorCVC {
    using cfloat = std::complex<float>;

    std::size_t fft_len{0};
    std::vector<std::vector<int>>  occupied;
    std::vector<std::vector<int>>  pilot_carriers;
    std::vector<std::vector<cfloat>> pilot_symbols;
    std::vector<std::vector<cfloat>> sync_words;
    bool output_is_shifted{true};

    void start(std::size_t N,
               const std::vector<std::vector<int>>& occupied_carriers,
               const std::vector<std::vector<int>>& pilot_carriers_in,
               const std::vector<std::vector<cfloat>>& pilot_symbols_in,
               const std::vector<std::vector<cfloat>>& sync_words_in,
               bool shifted = true)
    {
        if (N == 0) throw std::invalid_argument("fft_len must be > 0");
        if (occupied_carriers.empty())
            throw std::invalid_argument("occupied carriers empty");
        if (pilot_carriers_in.size() != pilot_symbols_in.size())
            throw std::invalid_argument("pilot carriers/symbols size mismatch");
        for (std::size_t i = 0; i < pilot_carriers_in.size(); ++i)
            if (pilot_carriers_in[i].size() != pilot_symbols_in[i].size())
                throw std::invalid_argument("pilot carriers/symbols inner size mismatch");
        for (const auto& sw : sync_words_in)
            if (sw.size() != N)
                throw std::invalid_argument("sync word length != fft_len");

        fft_len = N;
        occupied       = occupied_carriers;
        pilot_carriers = pilot_carriers_in;
        pilot_symbols  = pilot_symbols_in;
        sync_words     = sync_words_in;
        output_is_shifted = shifted;
    }

    void stop() {
        fft_len = 0;
        occupied.clear();
        pilot_carriers.clear();
        pilot_symbols.clear();
        sync_words.clear();
    }

    void map_frame(const std::vector<cfloat>& data_in,
                   std::vector<std::vector<cfloat>>& out) const
    {
        if (fft_len == 0) return;

        for (const auto& sw : sync_words) out.push_back(sw);

        const std::size_t occ_sets = occupied.size();
        const std::size_t pilot_sets = pilot_carriers.size();
        const int N  = static_cast<int>(fft_len);
        const int Nh = N / 2;

        auto map_pos = [&](int bin)->std::size_t {
            int pos = 0;
            if (output_is_shifted) {
                int t = bin + Nh; t %= N; if (t < 0) t += N;
                pos = t;
            } else {
                int t = bin % N; if (t < 0) t += N;
                pos = t;
            }
            return static_cast<std::size_t>(pos);
        };

        std::size_t cursor = 0;
        std::size_t sym_idx = 0;
        while (cursor < data_in.size()) {
            std::vector<cfloat> sym(fft_len, cfloat{0.f, 0.f});

            const auto& occ = occupied[sym_idx % occ_sets];
            for (std::size_t i = 0; i < occ.size(); ++i) {
                if (cursor >= data_in.size()) break;
                sym[map_pos(occ[i])] = data_in[cursor++];
            }

            if (!pilot_carriers.empty()) {
                const auto& pcs = pilot_carriers[sym_idx % pilot_sets];
                const auto& pss = pilot_symbols[sym_idx % pilot_sets];
                for (std::size_t i = 0; i < pcs.size(); ++i) {
                    sym[map_pos(pcs[i])] = pss[i];
                }
            }

            out.push_back(std::move(sym));
            ++sym_idx;
        }
    }
};

} // namespace gr::digital

#endif // GNURADIO_DIGITAL_OFDM_CARRIERALLOCATOR_HPP
