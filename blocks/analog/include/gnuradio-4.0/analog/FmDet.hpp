#ifndef GR_BLOCKS_ANALOG_FMDET_HPP_
#define GR_BLOCKS_ANALOG_FMDET_HPP_

#include <complex>
#include <numbers>
#include <algorithm>
#include <variant>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

namespace gr::blocks::analog {

template<typename T> struct FmDet;

template<>
struct FmDet<std::complex<float>>
    : Block<FmDet<std::complex<float>>>
{
    using Description = Doc<"IQ slope detector (complex → float)">;

    PortIn <std::complex<float>> in;
    PortOut<float>               out;

    Annotated<float, "samplerate", Visible> samplerate { 1.0f };
    Annotated<float, "freq_low",   Visible> f_low      { -1.0f };
    Annotated<float, "freq_high",  Visible> f_high     {  1.0f };
    Annotated<float, "scl",        Visible> scl        {  1.0f };

    GR_MAKE_REFLECTABLE(FmDet, in, out, samplerate, f_low, f_high, scl);

    FmDet() = default;

    explicit FmDet(property_map pm)
    {
        if (auto it = pm.find("scl"); it != pm.end())
            try { scl = std::get<float>(it->second); } catch (...) {}
        if (auto lo = pm.find("freq_low");  lo != pm.end())
            try { f_low  = std::get<float>(lo->second); } catch (...) {}
        if (auto hi = pm.find("freq_high"); hi != pm.end())
            try { f_high = std::get<float>(hi->second); } catch (...) {}
        _recompute_bias();
    }

    void start()               // reset every run
    {
        _prev  = {1.0f, 0.0f};
        _recompute_bias();
    }

    void  set_scale(float s)        { scl = s;  _recompute_bias(); }
    float scale()            const { return scl; }

    void  set_freq_range(float lo, float hi)
    { f_low = lo; f_high = hi; _recompute_bias(); }

    float freq_low () const { return f_low;  }
    float freq_high() const { return f_high; }
    float bias()      const { return _bias;  }

    work::Status processOne(const std::complex<float>& x, float& y)
    {
        const std::complex<float> prod = x * std::conj(_prev);
        _prev = x;
        y = scl * std::arg(prod) - _bias;
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

private:
    std::complex<float> _prev {1.0f, 0.0f};
    float               _bias {0.0f};

    void _recompute_bias()
    {
        const float hi = f_high, lo = f_low;
        _bias = (hi != lo) ? 0.5f * scl * (hi + lo) / (hi - lo) : 0.0f;
    }
};

using FmDetC = FmDet<std::complex<float>>;

GR_REGISTER_BLOCK("gr::blocks::analog::FmDet",
                  gr::blocks::analog::FmDetC)

} // namespace gr::blocks::analog
#endif /* GR_BLOCKS_ANALOG_FMDET_HPP_ */
