#pragma once

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <vector>

namespace gr::testing {

GR_REGISTER_BLOCK("gr::incubator::basic::VectorSink", // Name
    gr::testing::VectorSink, ([T]),                   // Struct, and type T
    // [ uint8_t, int16_t, int32_t, float, double, std::complex<float>, std::complex<double> ] // Supported Types
    [ uint8_t, uint16_t, uint32_t, uint64_t, int8_t, int16_t, int32_t, int64_t, float, double, std::complex<float>, std::complex<double>, std::string, gr::Packet<float>, gr::Packet<double>, gr::Tensor<float>, gr::Tensor<double>, gr::DataSet<float>, gr::DataSet<double> ])

using namespace gr;

template<typename T>
struct VectorSink : Block<VectorSink<T>> {
    using Description = Doc<"Accumulates all incoming samples into an internal std::vector<T>. "
                            "After the graph finishes, call data() to retrieve the collected sequence for inspection or assertion. "
                            "max_samples (default 0 = unlimited) caps the buffer size; when the limit is reached the block "
                            "calls requestStop(), useful as a termination condition for graphs that run indefinitely. "
                            "start() clears the buffer so the sink can be reused across multiple graph runs. "
                            "Signal chain: [any source / processing chain] -> VectorSink.">;

    PortIn<T> in;

    Annotated<gr::Size_t, "max_samples", Doc<"Maximum samples to store (0 = unlimited)">> max_samples = gr::Size_t{0u};

    GR_MAKE_REFLECTABLE(VectorSink, in, max_samples);

    std::vector<T> _data;

    void start() { _data.clear(); }

    void processOne(T x) {
        const std::size_t lim = static_cast<std::size_t>(static_cast<gr::Size_t>(max_samples));
        if (lim == 0u || _data.size() < lim) {
            _data.push_back(x);
            if (lim > 0u && _data.size() >= lim) {
                this->requestStop();
            }
        }
    }

    [[nodiscard]] const std::vector<T>& data() const noexcept { return _data; }
};

} // namespace gr::testing
