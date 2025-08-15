// blocks/digital/test/mapping/qa_ChunksToSymbols.cpp
/*
 * Unit tests for the lean C++23 ChunksToSymbols mapper.
 * - Focus on processOne(..) as the primitive
 * - Cover 1D and multi-D, table updates, and corner cases
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/mapping/ChunksToSymbols.hpp>

#include <algorithm>
#include <complex>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include <numeric> // for iota if ever used

using namespace boost::ut;
using gr::digital::ChunksToSymbolsBF;
using gr::digital::ChunksToSymbolsBC;
using gr::digital::ChunksToSymbolsSF;
using gr::digital::ChunksToSymbolsSC;
using gr::digital::ChunksToSymbolsIF;
using gr::digital::ChunksToSymbolsIC;
using gr::digital::complexf;

const suite ChunksToSymbolsSuite = [] {
    "bf: basic 1D mapping"_test = [] {
        ChunksToSymbolsBF op;
        op.D = 1;
        op.table = {-3.f, -1.f, 1.f, 3.f};
        op.start();

        const std::uint8_t in[] = {0, 1, 2, 3, 3, 2, 1, 0};
        std::vector<float> out;
        for (auto i : in) {
            auto s = op.processOne(i);
            expect(s.size() == 1_u);
            out.push_back(s[0]);
        }

        std::vector<float> expected = {-3.f, -1.f, 1.f, 3.f, 3.f, 1.f, -1.f, -3.f};
        expect(out == expected);
    };

    "bc: basic 1D complex mapping"_test = [] {
        ChunksToSymbolsBC op;
        op.D = 1;
        op.table = {complexf{1, 0}, complexf{0, 1}, complexf{-1, 0}, complexf{0, -1}};
        op.start();

        const std::uint8_t in[] = {0, 1, 2, 3, 3, 2, 1, 0};
        std::vector<complexf> out;
        for (auto i : in) {
            auto s = op.processOne(i);
            out.push_back(s[0]);
        }

        std::vector<complexf> expected = {complexf{1, 0},
                                          complexf{0, 1},
                                          complexf{-1, 0},
                                          complexf{0, -1},
                                          complexf{0, -1},
                                          complexf{-1, 0},
                                          complexf{0, 1},
                                          complexf{1, 0}};
        expect(out == expected);
    };

    "bf: 2D mapping"_test = [] {
        const unsigned maxval = 4;
        const unsigned D = 2;

        ChunksToSymbolsBF op;
        op.D = D;
        op.table.resize(maxval * D);
        for (unsigned i = 0; i < maxval * D; ++i) op.table[i] = static_cast<float>(i);
        op.start();

        std::vector<std::uint8_t> in(maxval);
        for (unsigned v = 0; v < maxval; ++v) in[v] = static_cast<std::uint8_t>((v * 13) % maxval);

        std::vector<float> out;
        op.processMany(in, out);

        std::vector<float> expected;
        for (auto d : in)
            for (unsigned k = 0; k < D; ++k)
                expected.push_back(static_cast<float>(static_cast<unsigned>(d) * D + k));

        expect(out == expected);
    };

    "bf: 3D mapping"_test = [] {
        const unsigned maxval = 8;
        const unsigned D = 3;

        ChunksToSymbolsBF op;
        op.D = D;
        op.table.resize(maxval * D);
        for (unsigned i = 0; i < maxval * D; ++i) op.table[i] = static_cast<float>(i);
        op.start();

        std::vector<std::uint8_t> in(maxval);
        for (unsigned v = 0; v < maxval; ++v) in[v] = static_cast<std::uint8_t>((v * 7) % maxval);

        std::vector<float> out;
        op.processMany(in.begin(), in.end(), std::back_inserter(out));

        std::vector<float> expected;
        for (auto d : in)
            for (unsigned k = 0; k < D; ++k)
                expected.push_back(static_cast<float>(static_cast<unsigned>(d) * D + k));

        expect(out == expected);
    };

    "update: set_symbol_table + restart"_test = [] {
        ChunksToSymbolsSF op;
        op.D = 1;
        op.table = {-3.f, -1.f, 1.f, 3.f};
        op.start();

        std::vector<std::int16_t> in = {0, 1, 2, 3};
        std::vector<float> outA;
        op.processMany(in, outA);

        op.set_symbol_table({12.f, -12.f, 6.f, -6.f});
        op.start(); // recompute arity

        std::vector<float> outB;
        op.processMany(in, outB);

        expect(outA == std::vector<float>({-3.f, -1.f, 1.f, 3.f}));
        expect(outB == std::vector<float>({12.f, -12.f, 6.f, -6.f}));
    };

    "errors: start() validation"_test = [] {
        // D == 0
        {
            ChunksToSymbolsBF op;
            op.D = 0;
            op.table = {1.f, 2.f};
            expect(throws<std::invalid_argument>([&] { op.start(); }));
        }
        // empty table
        {
            ChunksToSymbolsBF op;
            op.D = 1;
            op.table.clear();
            expect(throws<std::invalid_argument>([&] { op.start(); }));
        }
        // size not multiple of D
        {
            ChunksToSymbolsBF op;
            op.D = 2;
            op.table = {1.f, 2.f, 3.f}; // 3 % 2 != 0
            expect(throws<std::invalid_argument>([&] { op.start(); }));
        }
    };

    "errors: processOne bounds (signed/unsigned)"_test = [] {
        // unsigned: index >= arity
        {
            ChunksToSymbolsBF op;
            op.D = 1;
            op.table = {0.f, 1.f};
            op.start();
            expect(throws<std::out_of_range>([&] { (void)op.processOne(static_cast<std::uint8_t>(2)); }));
        }
        // signed: negative
        {
            ChunksToSymbolsSF op;
            op.D = 1;
            op.table = {0.f, 1.f};
            op.start();
            expect(throws<std::out_of_range>([&] { (void)op.processOne(static_cast<std::int16_t>(-1)); }));
        }
    };

    "ic/if/sc flavors compile & map 1D"_test = [] {
        // IC
        {
            ChunksToSymbolsIC op;
            op.D = 1;
            op.table = {complexf{1, 0}, complexf{0, 1}, complexf{-1, 0}, complexf{0, -1}};
            op.start();
            std::int32_t in = 2;
            auto span = op.processOne(in);
            expect(span.size() == 1_u);
            expect(span[0] == complexf{-1, 0});
        }
        // IF
        {
            ChunksToSymbolsIF op;
            op.D = 1;
            op.table = {-3.f, -1.f, 1.f, 3.f};
            op.start();
            std::int32_t in = 3;
            auto span = op.processOne(in);
            expect(span[0] == 3.f);
        }
        // SC
        {
            ChunksToSymbolsSC op;
            op.D = 1;
            op.table = {complexf{-3, 1}, complexf{-1, -1}, complexf{1, 1}, complexf{3, -1}};
            op.start();
            std::int16_t in = 1;
            auto span = op.processOne(in);
            expect(span[0] == complexf{-1, -1});
        }
    };

    "consistency: processMany equals repeated processOne"_test = [] {
        ChunksToSymbolsBF op;
        op.D = 3;
        const unsigned A = 5;
        op.table.resize(A * op.D);
        for (unsigned i = 0; i < A * op.D; ++i) op.table[i] = static_cast<float>(i);
        op.start();

        std::vector<std::uint8_t> in = {0, 2, 4, 1, 3};
        std::vector<float> out_many;
        op.processMany(in, out_many);

        std::vector<float> out_one;
        for (auto idx : in) {
            auto s = op.processOne(idx);
            out_one.insert(out_one.end(), s.begin(), s.end());
        }
        expect(out_many == out_one);
    };
};

int main() {}
