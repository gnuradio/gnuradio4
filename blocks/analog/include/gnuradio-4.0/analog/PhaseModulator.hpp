#ifndef GR_BLOCKS_ANALOG_PHASEMODULATOR_HPP_
#define GR_BLOCKS_ANALOG_PHASEMODULATOR_HPP_

#include <cmath>
#include <complex>
#include <numbers>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

namespace gr::blocks::analog {

struct PhaseModulator : Block<PhaseModulator>
{
    using Description = Doc<"Phase modulator (float → complex<float>)">;

    PortIn<float>                in;
    PortOut<std::complex<float>> out;

    Annotated<float,
              "sensitivity",
              Visible,
              Doc<"Phase change [rad] per input-unit">>
        sensitivity { 1.0f };

    GR_MAKE_REFLECTABLE(PhaseModulator, in, out, sensitivity);

    explicit PhaseModulator(property_map) {}

    work::Status processOne(float x, std::complex<float>& y)
    {
        const float φ = x * sensitivity;
        y = { std::cos(φ), std::sin(φ) };
        return work::Status::OK;
    }

    template<InputSpanLike  InSpan,
             OutputSpanLike OutSpan>
    work::Status processBulk(const InSpan& xs, OutSpan& ys)
    {
        const std::size_t n = std::min(xs.size(), ys.size());
        for (std::size_t i = 0; i < n; ++i)
            processOne(xs[i], ys[i]);
        ys.publish(n);
        return work::Status::OK;
    }
};

using PhaseModC = PhaseModulator;

GR_REGISTER_BLOCK("gr::blocks::analog::PhaseModulator",
                  gr::blocks::analog::PhaseModulator)

} // namespace gr::blocks::analog
#endif /* GR_BLOCKS_ANALOG_PHASEMODULATOR_HPP_ */
