#ifndef GR_BLOCKS_ANALOG_AGC2_HPP_
#define GR_BLOCKS_ANALOG_AGC2_HPP_

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <type_traits>

namespace gr::blocks::analog {

template<typename T>
class Agc2 : public Block<Agc2<T>>
{
public:
    Annotated<float, "attack_rate", Visible> attack_rate = 1.0e-1f;
    Annotated<float, "decay_rate",  Visible> decay_rate  = 1.0e-2f;
    Annotated<float, "reference",   Visible> reference   = 1.0f;
    Annotated<float, "max_gain",    Visible> max_gain    = 0.0f;   // 0 ⇒ unlimited

    PortIn<T>  in;
    PortOut<T> out;

    GR_MAKE_REFLECTABLE(Agc2, in, out, attack_rate, decay_rate, reference, max_gain);

    static constexpr bool supports_selftest = true;

    void start() { _gain = 1.0f; }

    work::Status processOne(const T& x, T& y)
    {
        if (!std::isfinite(_gain)) _gain = 1.0f;

        y = static_cast<T>(x * _gain);
        const float amp = amplitude(y);

        const float err  = reference - amp;              // positive if too quiet
        const float rate = (err < 0.0f) ? float(attack_rate)   // too loud → attack
                                         : float(decay_rate);  // too quiet → decay

        _gain += rate * err;

        /* clamp gain */
        if (max_gain > 0.0f && _gain > max_gain) _gain = max_gain;
        if (_gain < 1.0e-5f)                      _gain = 1.0e-5f;

        return work::Status::OK;
    }

    template<InputSpanLike InSpan, OutputSpanLike OutSpan>
    work::Status processBulk(const InSpan& xs, OutSpan& ys)
    {
        const std::size_t n = std::min(xs.size(), ys.size());
        for (std::size_t i = 0; i < n; ++i) processOne(xs[i], ys[i]);
        ys.publish(n);
        return work::Status::OK;
    }

    explicit Agc2(property_map) {}

private:
    static inline float amplitude(float v)                      { return std::fabs(v); }
    static inline float amplitude(const std::complex<float>& v) { return std::abs(v);  }

    float _gain {1.0f};
};

using Agc2FF = Agc2<float>;
using Agc2CC = Agc2<std::complex<float>>;

GR_REGISTER_BLOCK("gr::blocks::analog::Agc2FF", Agc2FF)
GR_REGISTER_BLOCK("gr::blocks::analog::Agc2CC", Agc2CC)

} // namespace gr::blocks::analog
#endif // GR_BLOCKS_ANALOG_AGC2_HPP_
