#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/mapping/DiffCoding.hpp>
#include <vector>

using namespace boost::ut;
using namespace gr::digital;

static const suite DiffCodingSuite = [] {
    "roundtrip: various moduli & seeds"_test = [] {
        for (unsigned M : {2u, 4u, 8u, 17u}) {
            for (std::uint32_t seed : {0u, 3u}) {
                DiffEncoder enc;
                DiffDecoder dec;
                enc.start(M, seed);
                dec.start(M, seed);

                std::vector<std::uint32_t> in  = {0,1,2,3,3,2,1,0,5,9,11,15};
                std::vector<std::uint32_t> out; out.reserve(in.size());

                for (auto v : in) {
                    auto e = enc.processOne(v);
                    auto d = dec.processOne(e);
                    out.push_back(d % M);
                }

                expect(out.size() == in.size());
                for (std::size_t i = 0; i < in.size(); ++i)
                    expect(eq(out[i], in[i] % M)) << "i=" << i;
            }
        }
    };

    "invalid modulus throws"_test = [] {
        DiffEncoder enc; DiffDecoder dec;
        expect(throws([&]{ enc.start(0); }));
        expect(throws([&]{ dec.start(1); }));
    };
};
