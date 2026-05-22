#pragma once

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <vector>

namespace gr::testing {

GR_REGISTER_BLOCK("gr::testing::VectorSource", // Name
    gr::testing::VectorSource, ([T]),          // type and type
    [ uint8_t, uint16_t, uint32_t, uint64_t, int8_t, int16_t, int32_t, int64_t, float, double, std::complex<float>, std::complex<double>, std::string, gr::Packet<float>, gr::Packet<double>, gr::Tensor<float>, gr::Tensor<double>, gr::DataSet<float>, gr::DataSet<double> ])

using namespace gr;

template<typename T>
struct VectorSource : Block<VectorSource<T>> {
    using Description = Doc<"Replays a fixed sequence of samples, then stops the graph. "
                            "Emits data[0], data[1], ..., data[N-1] in order, then calls requestStop() so "
                            "the scheduler tears down the graph automatically. Output is produced in chunks "
                            "matched to the downstream port's buffer size. "
                            "Primary use: unit tests and demos — inject a deterministic signal without real hardware. "
                            "Call start() to reset the read position and re-run the scheduler for repeated playback. "
                            "Signal chain: VectorSource -> [processing chain] -> VectorSink.">;

    PortOut<T> out;

    Annotated<std::vector<T>, "data", Doc<"Samples to emit in order.">> data{};

    GR_MAKE_REFLECTABLE(VectorSource, out, data);

    std::size_t _pos{0u};

    void start() { _pos = 0u; }

    [[nodiscard]] work::Status processBulk(OutputSpanLike auto& outSpan) {
        const auto& d = static_cast<const std::vector<T>&>(data);
        if (_pos >= d.size()) {
            outSpan.publish(0u);
            return work::Status::DONE;
        }
        const std::size_t nToPublish = std::min(d.size() - _pos, outSpan.size());
        if (nToPublish == 0u) {
            return work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        std::copy_n(d.begin() + static_cast<std::ptrdiff_t>(_pos), nToPublish, outSpan.begin());
        _pos += nToPublish;
        outSpan.publish(nToPublish);
        return _pos >= d.size() ? work::Status::DONE : work::Status::OK;
    }
};

} // namespace gr::testing
