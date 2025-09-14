#ifndef GNURADIO_DIGITAL_TIMING_SYMBOLSYNC_HPP
#define GNURADIO_DIGITAL_TIMING_SYMBOLSYNC_HPP

#include <complex>
#include <type_traits>
#include <cstddef>
#include <algorithm>
#include <cmath>

namespace gr::digital {

template <typename T>
class SymbolSync {
public:
    SymbolSync() = default;

    void start(float sps,
               float /*loop_bw*/ = 0.0f,
               float /*damping*/ = 1.0f,
               float /*ted_gain*/ = 1.0f,
               float /*max_dev*/ = 1.5f,
               int   osps = 1)
    {
        sps0_ = std::max(1e-6f, sps);
        sps_  = sps0_;
        mu_   = 0.0f;          // phase accumulator in [0, sps)
        osps_ = std::max(1, osps);
        prev_valid_ = false;
        last_ = T{};
        started_ = true;
    }

    void stop() { started_ = false; prev_valid_ = false; }

    bool processOne(const T& x, T& y)
    {
        if (!started_) return false;

        if (!prev_valid_) {
            prev_ = x;
            prev_valid_ = true;
            mu_ += 1.0f;       
            return false;
        }

        mu_ += 1.0f;           

        bool emit = false;
        if (mu_ >= sps_) {
            const float overshoot = mu_ - sps_;
            const float t = 1.0f - overshoot;
            y = lerp(prev_, x, t);
            mu_ -= sps_;
            emit = true;
        }

        prev_ = x;
        return emit;
    }

private:
    static T lerp(const T& a, const T& b, float t)
    {
        if constexpr (std::is_same_v<T, float>) {
            return a + t * (b - a);
        } else {
            return T(a.real() + t * (b.real() - a.real()),
                     a.imag() + t * (b.imag() - a.imag()));
        }
    }

    float sps0_{2.0f};   
    float sps_{2.0f};    
    float mu_{0.0f};     
    int   osps_{1};      
    bool  started_{false};

    T prev_{};
    T last_{};
    bool prev_valid_{false};
};

using SymbolSyncf  = SymbolSync<float>;
using SymbolSynccf = SymbolSync<std::complex<float>>;

} // namespace gr::digital

#endif // GNURADIO_DIGITAL_TIMING_SYMBOLSYNC_HPP
