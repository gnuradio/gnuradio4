#ifndef GNURADIO_DIGITAL_MAPBB_HPP
#define GNURADIO_DIGITAL_MAPBB_HPP

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace gr::digital {

struct MapBB {
    std::vector<std::uint32_t> table;

    void start(const std::vector<std::uint32_t>& map) {
        if (map.empty()) throw std::invalid_argument("MapBB: map must not be empty");
        table = map;
    }

    void stop() {}

    std::uint32_t processOne(std::uint32_t in) const {
        if (in >= table.size()) throw std::out_of_range("MapBB: index out of range");
        return table[in];
    }
};

} // namespace gr::digital

#endif // GNURADIO_DIGITAL_MAPBB_HPP
