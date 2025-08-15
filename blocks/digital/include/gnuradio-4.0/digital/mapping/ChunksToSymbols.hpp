#ifndef GR4_DIGITAL_MAPPING_CHUNKS_TO_SYMBOLS_HPP
#define GR4_DIGITAL_MAPPING_CHUNKS_TO_SYMBOLS_HPP

#include <complex>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>
#include <algorithm>

namespace gr::digital {

using complexf = std::complex<float>;

template <class IN_T, class OUT_T>
struct ChunksToSymbols {
    unsigned D{1};                 // dimensions per symbol
    std::vector<OUT_T> table{};    // size must be arity * D after start()
    unsigned arity{0};             // derived in start()

    mutable std::vector<OUT_T> scratch{};

    void start()
    {
        if (D == 0) throw std::invalid_argument("ChunksToSymbols: D must be >= 1");
        if (table.empty()) throw std::invalid_argument("ChunksToSymbols: empty table");
        if (table.size() % D != 0)
            throw std::invalid_argument("ChunksToSymbols: table size must be multiple of D");
        arity = static_cast<unsigned>(table.size() / D);
        scratch.resize(D);
    }

    void stop() {}

    void set_symbol_table(const std::vector<OUT_T>& new_table) { table = new_table; }

    std::span<const OUT_T> processOne(IN_T idx)
    {
        long long v = static_cast<long long>(idx);
        if constexpr (std::is_signed_v<IN_T>) {
            if (v < 0) throw std::out_of_range("ChunksToSymbols: negative index");
        }
        const std::size_t u = static_cast<std::size_t>(v);
        if (u >= arity) throw std::out_of_range("ChunksToSymbols: index >= arity");
        const std::size_t base = u * static_cast<std::size_t>(D);
        for (unsigned k = 0; k < D; ++k) scratch[k] = table[base + k];
        return std::span<const OUT_T>(scratch.data(), D);
    }

    void processMany(const std::vector<IN_T>& in, std::vector<OUT_T>& out)
    {
        out.reserve(out.size() + in.size() * static_cast<std::size_t>(D));
        for (auto idx : in) {
            auto s = processOne(idx);
            out.insert(out.end(), s.begin(), s.end());
        }
    }

    template <class InIt, class OutIt>
    void processMany(InIt first, InIt last, OutIt out_it)
    {
        for (; first != last; ++first) {
            auto s = processOne(*first);
            out_it = std::copy(s.begin(), s.end(), out_it);
        }
    }
};

using ChunksToSymbolsBF = ChunksToSymbols<std::uint8_t, float>;
using ChunksToSymbolsBC = ChunksToSymbols<std::uint8_t, complexf>;
using ChunksToSymbolsSF = ChunksToSymbols<std::int16_t, float>;
using ChunksToSymbolsSC = ChunksToSymbols<std::int16_t, complexf>;
using ChunksToSymbolsIF = ChunksToSymbols<std::int32_t, float>;
using ChunksToSymbolsIC = ChunksToSymbols<std::int32_t, complexf>;

} // namespace gr::digital

#endif // GR4_DIGITAL_MAPPING_CHUNKS_TO_SYMBOLS_HPP
