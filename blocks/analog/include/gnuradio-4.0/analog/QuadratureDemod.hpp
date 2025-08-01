#ifndef GR_BLOCKS_ANALOG_QUADRATUREDEMOD_HPP_
#define GR_BLOCKS_ANALOG_QUADRATUREDEMOD_HPP_

#include <cmath>
#include <complex>
#include <numbers>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

namespace gr::blocks::analog {

struct QuadratureDemod : Block<QuadratureDemod>
{
    using Description = Doc<"Quadrature FM demodulator (complex → float)">;

    PortIn<std::complex<float>> in;
    PortOut<float>              out;

    Annotated<float,
              "gain",
              Visible,
              Doc<"Output scale (rad → user units)">>
        gain { 1.0f };

    GR_MAKE_REFLECTABLE(QuadratureDemod, in, out, gain);

    explicit QuadratureDemod(property_map init)
    {
        if (auto it = init.find("gain"); it != init.end()) {
            gain = std::get<float>(it->second);
        }
    }

    void start()
    {
        _have_prev = false;
        _prev      = { 1.0f, 0.0f };
    }

    work::Status processOne(std::complex<float> x, float& y)
    {
        if (!_have_prev) {
            y          = 0.0f;   // undefined → 0
            _have_prev = true;
        } else {
            y = gain * std::arg(x * std::conj(_prev));
        }
        _prev = x;
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
    std::complex<float> _prev      { 1.0f, 0.0f };
    bool                _have_prev{ false };
};

GR_REGISTER_BLOCK("gr::blocks::analog::QuadratureDemod",
                  gr::blocks::analog::QuadratureDemod)

} // namespace gr::blocks::analog
#endif /* GR_BLOCKS_ANALOG_QUADRATUREDEMOD_HPP_ */
