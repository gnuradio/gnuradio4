#ifndef GR_BLOCKS_ANALOG_FREQUENCYMOD_HPP_
#define GR_BLOCKS_ANALOG_FREQUENCYMOD_HPP_

#include <cmath>
#include <complex>
#include <numbers>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

namespace gr::blocks::analog {

template<typename T> struct FrequencyMod;           // primary template

template<>
struct FrequencyMod<float> : Block<FrequencyMod<float>>
{
    using Description = Doc<"Frequency-modulator (float → complex<float>)">;

    PortIn<float>                 in;
    PortOut<std::complex<float>>  out;

    Annotated<float,
              "sensitivity",
              Visible,
              Doc<"Phase increment [rad/sample] per input-unit">>
        sensitivity { std::numbers::pi_v<float> / 4.0f };

    GR_MAKE_REFLECTABLE(FrequencyMod, in, out, sensitivity);

    explicit FrequencyMod(property_map) {}

    void start()                 // *no* override – base has no start()
    {
        _phase = 0.0f;
    }

    work::Status processOne(float x, std::complex<float>& y)
    {
        _phase = std::remainder(_phase + x * sensitivity,
                                2.0f * std::numbers::pi_v<float>);
        y = { std::cos(_phase), std::sin(_phase) };
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

private:
    float _phase { 0.0f };
};

using FrequencyModF = FrequencyMod<float>;

GR_REGISTER_BLOCK("gr::blocks::analog::FrequencyMod",
                  gr::blocks::analog::FrequencyMod, ([T]), [ float ])

} // namespace gr::blocks::analog
#endif /* GR_BLOCKS_ANALOG_FREQUENCYMOD_HPP_ */
