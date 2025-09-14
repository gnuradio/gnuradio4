#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/mapping/MapBB.hpp>
#include <vector>

using namespace boost::ut;
using namespace gr::digital;

static const suite MapBBSuite = [] {
    "simple lookup"_test = [] {
        MapBB m; m.start({10, 20, 30, 40});
        expect(m.processOne(0) == 10_u);
        expect(m.processOne(1) == 20_u);
        expect(m.processOne(3) == 40_u);
    };

    "values as in GR3 tests"_test = [] {
        MapBB m; m.start({7u, 31u, 128u, 255u});
        std::vector<unsigned> src = {0,1,2,3,0,1,2,3};
        std::vector<unsigned> expected = {7,31,128,255,7,31,128,255};
        for (std::size_t i = 0; i < src.size(); ++i) {
            expect(eq(m.processOne(src[i]), expected[i]));
        }
    };

    "empty/oor checks"_test = [] {
        MapBB m;
        expect(throws([&]{ m.start({}); }));
        m.start({1u});
        expect(throws([&]{ (void)m.processOne(5u); }));
    };
};
