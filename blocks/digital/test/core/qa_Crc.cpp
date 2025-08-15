// blocks/digital/test/core/qa_Crc.cpp
#include <boost/ut.hpp>
#include <gnuradio-4.0/digital/core/Crc.hpp>
#include <vector>

using namespace boost::ut;
using gr::digital::Crc;
using gr::digital::CrcState;

static std::vector<std::uint8_t> seq_0_to_15() {
    std::vector<std::uint8_t> v(16);
    for (std::uint8_t i = 0; i < 16; ++i) v[i] = i;
    return v;
}

const suite CrcSuite = [] {
    "CRC16-CCITT-Zero"_test = [] {
        Crc crc;
        crc.st = CrcState{16, 0x1021, 0x0000, 0x0000, false, false};
        crc.start();
        auto data = seq_0_to_15();
        auto out = crc.compute(data.data(), data.size());
        expect(out == 0x513D);
    };

    "CRC16-CCITT-False"_test = [] {
        Crc crc;
        crc.st = CrcState{16, 0x1021, 0xFFFF, 0x0000, false, false};
        crc.start();
        auto data = seq_0_to_15();
        auto out = crc.compute(data.data(), data.size());
        expect(out == 0x3B37);
    };

    "CRC16-CCITT-X25 (refin+refout)"_test = [] {
        Crc crc;
        crc.st = CrcState{16, 0x1021, 0xFFFF, 0xFFFF, true, true};
        crc.start();
        auto data = seq_0_to_15();
        auto out = crc.compute(data.data(), data.size());
        expect(out == 0x13E9);
    };

    "CRC32 (refin+refout)"_test = [] {
        Crc crc;
        crc.st = CrcState{32, 0x04C11DB7u, 0xFFFFFFFFu, 0xFFFFFFFFu, true, true};
        crc.start();
        auto data = seq_0_to_15();
        auto out = crc.compute(data.data(), data.size());
        expect(out == 0xCECEE288);
    };

    "CRC32C (refin+refout)"_test = [] {
        Crc crc;
        crc.st = CrcState{32, 0x1EDC6F41u, 0xFFFFFFFFu, 0xFFFFFFFFu, true, true};
        crc.start();
        auto data = seq_0_to_15();
        auto out = crc.compute(data.data(), data.size());
        expect(out == 0xD9C908EB);
    };

    "processOne matches compute"_test = [] {
        Crc a, b;
        auto data = seq_0_to_15();

        a.st = CrcState{32, 0x04C11DB7u, 0xFFFFFFFFu, 0xFFFFFFFFu, true, true};
        b.st = a.st;
        a.start(); b.start();

        for (auto byte : data) a.processOne(byte);

        auto reg = a.reg & a.mask;
        if (a.st.input_reflected != a.st.result_reflected)
            reg = Crc::reflect(reg, a.st.num_bits);
        auto res_stream = (reg ^ a.st.final_xor) & a.mask;

        auto res_compute = b.compute(data.data(), data.size());
        expect(res_stream == res_compute);
    };
};
