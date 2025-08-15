/* -*- c++ -*- */
/*
 * Lean Chunks→Symbols mapper (KISS, C++23)
 * - start()/stop() lifecycle (no custom constructors)
 * - processOne(..) is the single primitive; batched helpers build on it
 * - No virtuals/vtables; just a small POD-ish utility
 * - Works for 1D and D>1 (symbol is D consecutive entries)
 *
 * Usage sketch:
 *   ChunksToSymbolsBF op;
 *   op.D = 2;
 *   op.table = { … A×D entries … }; // e.g.,  {r0,i0, r1,i1, ...}
 *   op.start();
 *   auto span = op.processOne(3); // returns 2 values for index 3
 *   // for bulk: op.processMany(input, output);
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
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
    // Public, simple data (KISS)
    unsigned D{1};                 // dimensions per symbol
    std::vector<OUT_T> table{};    // size must be arity * D after start()
    unsigned arity{0};             // derived in start()

    // Scratch used by processOne; resized in start()
    mutable std::vector<OUT_T> scratch{};

    // Lifecycle
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

    // Change symbol table (call start() afterward to re-derive arity/scratch)
    void set_symbol_table(const std::vector<OUT_T>& new_table) { table = new_table; }

    // Primitive: map a single index to D outputs
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

    // Convenience: map many indices into an output vector (appends)
    void processMany(const std::vector<IN_T>& in, std::vector<OUT_T>& out)
    {
        out.reserve(out.size() + in.size() * static_cast<std::size_t>(D));
        for (auto idx : in) {
            auto s = processOne(idx);
            out.insert(out.end(), s.begin(), s.end());
        }
    }

    // Iterator-based variant
    template <class InIt, class OutIt>
    void processMany(InIt first, InIt last, OutIt out_it)
    {
        for (; first != last; ++first) {
            auto s = processOne(*first);
            out_it = std::copy(s.begin(), s.end(), out_it);
        }
    }
};

// Type aliases for classic flavors
using ChunksToSymbolsBF = ChunksToSymbols<std::uint8_t, float>;
using ChunksToSymbolsBC = ChunksToSymbols<std::uint8_t, complexf>;
using ChunksToSymbolsSF = ChunksToSymbols<std::int16_t, float>;
using ChunksToSymbolsSC = ChunksToSymbols<std::int16_t, complexf>;
using ChunksToSymbolsIF = ChunksToSymbols<std::int32_t, float>;
using ChunksToSymbolsIC = ChunksToSymbols<std::int32_t, complexf>;

} // namespace gr::digital

#endif // GR4_DIGITAL_MAPPING_CHUNKS_TO_SYMBOLS_HPP
