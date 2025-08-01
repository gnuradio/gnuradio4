#ifndef GR_BLOCKS_ANALOG_AGC_HPP_
#define GR_BLOCKS_ANALOG_AGC_HPP_

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <type_traits>

namespace gr::blocks::analog {

template<typename T>
class Agc : public Block<Agc<T>>
{
public:
    Annotated<float, "rate",      Visible> rate      = 1.0e-4f;
    Annotated<float, "reference", Visible> reference = 1.0f;
    Annotated<float, "max_gain",  Visible> max_gain  = 0.0f;   // 0 ⇒ unlimited

    PortIn<T>  in;
    PortOut<T> out;

    GR_MAKE_REFLECTABLE(Agc, in, out, rate, reference, max_gain);

    static constexpr bool supports_selftest = true;

    void start()              
    {
        _gain = 1.0f;
    }

    work::Status processOne(const T& x, T& y)
    {
        if (!std::isfinite(_gain))
            _gain = 1.0f;                     

        const float g = _gain;
        y = static_cast<T>(x * g);

        const float amp = amplitude(y);
        const float r   = std::clamp(float(rate), 0.0f, 1.0f);
        const float delta = (float(reference) - amp) * r;
        _gain = g + delta;

        if (max_gain > 0.0f && _gain > max_gain) _gain = max_gain;
        if (_gain < 1.0e-5f)                     _gain = 1.0e-5f;  // floor

        return work::Status::OK;
    }

    template<InputSpanLike InSpan, OutputSpanLike OutSpan>
    work::Status processBulk(const InSpan& xs, OutSpan& ys)
    {
        const std::size_t n = std::min(xs.size(), ys.size());
        for (std::size_t i = 0; i < n; ++i)
            processOne(xs[i], ys[i]);
        ys.publish(n);
        return work::Status::OK;
    }

    explicit Agc(property_map) {}

private:
    static inline float amplitude(float v)                      { return std::fabs(v); }
    static inline float amplitude(const std::complex<float>& v) { return std::abs(v);  }

    float _gain {1.0f};
};

using AgcFF = Agc<float>;
using AgcCC = Agc<std::complex<float>>;

GR_REGISTER_BLOCK("gr::blocks::analog::AgcFF", AgcFF)
GR_REGISTER_BLOCK("gr::blocks::analog::AgcCC", AgcCC)

} // namespace gr::blocks::analog
#endif // GR_BLOCKS_ANALOG_AGC_HPP_