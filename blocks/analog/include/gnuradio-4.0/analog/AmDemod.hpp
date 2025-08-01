#ifndef GR_BLOCKS_ANALOG_AM_DEMOD_HPP_
#define GR_BLOCKS_ANALOG_AM_DEMOD_HPP_

#include <cmath>
#include <complex>
#include <numbers>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

namespace gr::blocks::analog {

struct AmDemod : Block<AmDemod>
{
    PortIn<std::complex<float>> in;
    PortOut<float>              out;

    Annotated<float, "chan_rate",  Doc<"complex sample-rate [Hz]">>
        chan_rate  { 48'000.f };

    Annotated<float, "audio_pass", Doc<"audio LPF corner [Hz]">>
        audio_pass { 4'000.f };

    GR_MAKE_REFLECTABLE(AmDemod, in, out, chan_rate, audio_pass);

    explicit AmDemod(property_map) {}

    work::Status processOne(const std::complex<float>& x, float& y)
    {
        const float env = std::abs(x);
        const float alpha =
            std::exp(-2.f * std::numbers::pi_v<float> * audio_pass / chan_rate);
        _y = env + alpha * (_y - env);
        y  = _y;
        return work::Status::OK;
    }

    template<InputSpanLike  InSpan,
             OutputSpanLike OutSpan>
    work::Status processBulk(const InSpan& xs, OutSpan& ys)
    {
        const std::size_t n = std::min(xs.size(), ys.size());
        const float alpha =
            std::exp(-2.f * std::numbers::pi_v<float> * audio_pass / chan_rate);

        for (std::size_t i = 0; i < n; ++i) {
            const float env = std::abs(xs[i]);
            _y = env + alpha * (_y - env);
            ys[i] = _y;
        }
        ys.publish(n);
        return work::Status::OK;
    }

private:
    float _y { 0.f };   /* filter state */
};

GR_REGISTER_BLOCK("gr::blocks::analog::AmDemod",
                  gr::blocks::analog::AmDemod)

} // namespace gr::blocks::analog
#endif /* GR_BLOCKS_ANALOG_AM_DEMOD_HPP_ */
