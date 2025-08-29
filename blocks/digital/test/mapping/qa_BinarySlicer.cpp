#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/mapping/BinarySlicer.hpp>

using namespace boost::ut;
using namespace gr::digital;

static const suite BinarySlicerSuite = [] {
    "default threshold (0.0)"_test = [] {
        BinarySlicer s; s.start();
        expect(s.processOne(-1.0f) == 0_u);
        expect(s.processOne(-0.0001f) == 0_u);
        expect(s.processOne(0.0f) == 1_u);
        expect(s.processOne(0.7f) == 1_u);
    };

    "custom threshold"_test = [] {
        BinarySlicer s; s.start(0.5f);
        expect(s.processOne(0.49f) == 0_u);
        expect(s.processOne(0.5f) == 1_u);
        expect(s.processOne(0.51f) == 1_u);
    };
};
