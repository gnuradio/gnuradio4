#ifndef GNURADIO_GRAPH_UTILS_HPP
#define GNURADIO_GRAPH_UTILS_HPP

#include <cassert>
#include <complex>
#include <cstdint>
#include <cxxabi.h>
#include <format>
#include <map>
#include <new>
#include <numeric>
#include <print>
#include <ranges>
#include <string>
#include <string_view>
#include <tuple>
#include <typeinfo>
#include <unordered_map>

#if __has_include(<stdfloat>) && !defined(__ADAPTIVECPP__)
#include <stdfloat>
#else
#include <cstdint>
#include <limits>

// Inject into std only if truly unavailable (nonstandard, but pragmatic for compatibility)
namespace std {
using float32_t = float;
using float64_t = double;

static_assert(std::numeric_limits<float32_t>::is_iec559 && sizeof(float32_t) * 8 == 32, "float32_t must be a 32-bit IEEE 754 float");
static_assert(std::numeric_limits<float64_t>::is_iec559 && sizeof(float64_t) * 8 == 64, "float64_t must be a 64-bit IEEE 754 double");
} // namespace std
#endif

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <vir/simd.h>
#pragma GCC diagnostic pop

#ifndef DISABLE_SIMD
#define DISABLE_SIMD 0
#endif

namespace gr {

#pragma GCC diagnostic push // suppress unavoidable float/int/size_t conversion warnings
#pragma GCC diagnostic ignored "-Wconversion"
#ifdef __clang__
#pragma GCC diagnostic ignored "-Wimplicit-int-float-conversion"
#pragma GCC diagnostic ignored "-Wshorten-64-to-32"
#pragma GCC diagnostic ignored "-Wimplicit-int-float-conversion"
#pragma GCC diagnostic ignored "-Wimplicit-float-conversion"
#endif

using Size_t                                = std::uint32_t; // strict type definition in view of cross-platform/cross-compiler/cross-network portability similar to 'std::size_t' (N.B. which is not portable)
inline constexpr Size_t      max_Size       = std::numeric_limits<gr::Size_t>::max();
inline constexpr std::size_t max_size       = std::numeric_limits<std::size_t>::max();
inline constexpr Size_t      undefined_Size = std::numeric_limits<gr::Size_t>::max();
inline constexpr std::size_t undefined_size = std::numeric_limits<std::size_t>::max();

template<typename T, typename U>
T cast(U value) { /// gcc/clang warning suppressing cast
    return static_cast<T>(value);
}

#pragma GCC diagnostic pop

namespace meta {

struct null_type {};

template<typename... Ts>
struct print_types;

#if HAVE_SOURCE_LOCATION
[[gnu::always_inline]] constexpr void precondition(bool cond, const std::source_location loc = std::source_location::current()) {
    if consteval {
        if (not cond) {
            std::unreachable();
        }
    } else {
        struct handle {
            [[noreturn]] static void failure(std::source_location const& loc) {
                std::clog << "failed precondition in " << loc.file_name() << ':' << loc.line() << ':' << loc.column() << ": `" << loc.function_name() << "`\n";
                __builtin_trap();
            }
        };

        if (not cond) [[unlikely]] {
            handle::failure(loc);
        }
    }
}
#else
[[gnu::always_inline]] constexpr void precondition(bool cond) {
    if consteval {
        if (not cond) {
            std::unreachable();
        }
    } else {
        struct handle {
            [[noreturn]] static void failure() {
                std::println(stderr, "failed precondition");
                __builtin_trap();
            }
        };

        if (not cond) [[unlikely]] {
            handle::failure();
        }
    }
}
#endif

template<std::size_t N, typename CharT = char>
struct fixed_string {
    CharT _data[N + 1UZ] = {};

    // types
    using value_type      = CharT;
    using pointer         = value_type*;
    using const_pointer   = const value_type*;
    using reference       = value_type&;
    using const_reference = const value_type&;
    using size_type       = size_t;
    using difference_type = std::ptrdiff_t;

    // construction and assignment
    explicit fixed_string() = default;

    template<std::convertible_to<CharT>... Chars>
    requires(sizeof...(Chars) == N) and (... and not std::is_pointer_v<Chars>)
    constexpr explicit fixed_string(Chars... chars) noexcept //
        : _data{static_cast<CharT>(chars)..., '\0'} {}

    template<size_t... Is>
    requires(sizeof...(Is) == N)
    constexpr fixed_string(std::index_sequence<Is...>, const CharT* txt) noexcept //
        : _data{txt[Is]..., '\0'} {}

    consteval fixed_string(const CharT (&txt)[N + 1]) noexcept //
        : fixed_string(std::make_index_sequence<N>(), txt) {}

    template<typename It, std::sentinel_for<It> S>
    constexpr fixed_string(It begin, S end) //
        : fixed_string(std::make_index_sequence<N>(), std::to_address(begin)) {
        precondition(std::distance(begin, end) == N);
    }

    constexpr fixed_string(const fixed_string&) noexcept = default;

    constexpr fixed_string& operator=(const fixed_string&) noexcept = default;

    // capacity
    static constexpr std::integral_constant<size_type, N> size{};

    static constexpr std::integral_constant<size_type, N> length{};

    static constexpr std::integral_constant<size_type, N> max_size{};

    [[nodiscard]] static constexpr bool empty() noexcept { return N == 0; }

    // element access
    [[nodiscard]] constexpr reference operator[](size_type pos) { return _data[pos]; }

    [[nodiscard]] constexpr const_reference operator[](size_type pos) const { return _data[pos]; }

    [[nodiscard]] constexpr reference front() { return _data[0]; }

    [[nodiscard]] constexpr const_reference front() const { return _data[0]; }

    [[nodiscard]] constexpr reference back() { return _data[N - 1]; }

    [[nodiscard]] constexpr const_reference back() const { return _data[N - 1]; }

    // string operations
    [[nodiscard]] constexpr const_pointer c_str() const noexcept { return _data; }

    [[nodiscard]] constexpr const_pointer data() const noexcept { return _data; }

    [[nodiscard]] constexpr std::string_view view() const noexcept { return {_data, N}; }

    constexpr operator std::string_view() const noexcept { return {_data, N}; }

    [[nodiscard]] explicit operator std::string() const noexcept { return {_data, N}; }

    template<size_t N2>
    [[nodiscard]] constexpr friend fixed_string<N + N2, CharT> operator+(const fixed_string& lhs, const fixed_string<N2, CharT>& rhs) noexcept {
        return [&]<size_t... Is>(std::index_sequence<Is...>) { //
            return fixed_string<N + N2, CharT>{(Is < N ? lhs[Is] : rhs[Is - N])...};
        }(std::make_index_sequence<N + N2>());
    }

    [[nodiscard]] constexpr friend fixed_string<N + 1, CharT> operator+(const fixed_string& lhs, CharT rhs) noexcept {
        return [&]<size_t... Is>(std::index_sequence<Is...>) { //
            return fixed_string<N + 1, CharT>{(Is < N ? lhs[Is] : rhs)...};
        }(std::make_index_sequence<N + 1>());
    }

    [[nodiscard]] constexpr friend fixed_string<1 + N, CharT> operator+(const CharT lhs, const fixed_string& rhs) noexcept {
        return [&]<size_t... Is>(std::index_sequence<Is...>) { //
            return fixed_string<N + 1, CharT>{(Is < 1 ? lhs : rhs[Is - 1])...};
        }(std::make_index_sequence<N + 1>());
    }

    template<size_t N2>
    [[nodiscard]] constexpr friend fixed_string<N + N2 - 1, CharT> operator+(const fixed_string& lhs, const CharT (&rhs)[N2]) noexcept {
        return [&]<size_t... Is>(std::index_sequence<Is...>) { //
            return fixed_string<N + N2 - 1, CharT>{(Is < N ? lhs[Is] : rhs[Is - N])...};
        }(std::make_index_sequence<N + N2 - 1>());
    }

    template<size_t N1>
    [[nodiscard]] constexpr friend fixed_string<N1 + N - 1, CharT> operator+(const CharT (&lhs)[N1], const fixed_string& rhs) noexcept {
        return [&]<size_t... Is>(std::index_sequence<Is...>) { //
            return fixed_string<N1 + N - 1, CharT>{(Is < N1 - 1 ? lhs[Is] : rhs[Is - N1 + 1])...};
        }(std::make_index_sequence<N1 + N - 1>());
    }

    [[nodiscard]] constexpr size_t find_char(CharT c) const noexcept {
        for (size_t i = 0; i < N; ++i) {
            if (_data[i] == c) {
                return i;
            }
        }
        return N;
    }

    template<typename NewSize>
    [[nodiscard]] friend consteval fixed_string<NewSize::value, CharT> resize(const fixed_string& old, NewSize) noexcept {
        static_assert(NewSize::value <= N);
        return fixed_string<NewSize::value, CharT>(std::make_index_sequence<NewSize::value>(), old._data);
    }

    template<typename Offset, typename NewSize = std::integral_constant<size_t, N - Offset::value>>
    [[nodiscard]] friend consteval fixed_string<NewSize::value, CharT> substring(const fixed_string& old, Offset, NewSize = {}) noexcept {
        static_assert(Offset::value + NewSize::value <= N);
        static_assert(Offset::value >= 0);
        static_assert(NewSize::value >= 0);
        return fixed_string<NewSize::value, CharT>(std::make_index_sequence<NewSize::value>(), old._data + Offset::value);
    }

    // [fixed.string.comparison], non-member comparison functions
    [[nodiscard]] friend constexpr bool operator==(const fixed_string& lhs, const fixed_string& rhs) { //
        return lhs.view() == rhs.view();
    }

    template<std::size_t N2>
    [[nodiscard]] friend constexpr bool operator==(const fixed_string&, const fixed_string<N2, CharT>&) { //
        return false;
    }

    template<size_t N2>
    requires(N2 != N + 1)
    [[nodiscard]] friend constexpr bool operator==(const fixed_string&, const CharT (&)[N2]) { //
        return false;
    }

    [[nodiscard]] friend constexpr auto operator<=>(const fixed_string& lhs, const fixed_string& rhs) { //
        return lhs.view() <=> rhs.view();
    }

    template<size_t N2>
    requires(N2 != N)
    [[nodiscard]] friend constexpr auto operator<=>(const fixed_string& lhs, const fixed_string<N2, CharT>& rhs) { //
        return lhs.view() <=> rhs.view();
    }

    template<size_t N2>
    requires(N2 != N + 1)
    [[nodiscard]] friend constexpr auto operator<=>(const fixed_string& lhs, const CharT (&rhs)[N2]) { //
        return lhs.view() <=> std::string_view(rhs, rhs + N2 - 1);
    }
};

template<fixed_string S, typename T = std::remove_const_t<decltype(S)>>
class constexpr_string;

// fixed_string deduction guides
template<typename CharT, std::convertible_to<CharT>... Rest>
fixed_string(CharT, Rest...) -> fixed_string<1 + sizeof...(Rest), CharT>;

template<typename CharT, size_t N>
fixed_string(const CharT (&str)[N]) -> fixed_string<N - 1, CharT>;

template<auto S>
fixed_string(constexpr_string<S>) -> fixed_string<S.size, typename decltype(S)::value_type>;

// fixed_string trait
template<typename T>
struct is_fixed_string : std::false_type {};

template<typename CharT, std::size_t N>
struct is_fixed_string<gr::meta::fixed_string<N, CharT>> : std::true_type {};

template<typename T>
concept FixedString = is_fixed_string<T>::value;

/**
 * Store a compile-time string as a type, rather than a value. This enables:
 *
 * 1. Passing strings as function parameters and using them in constant expressions in the function body.
 *
 * 2. Conversion to C-String and std::string_view can return a never-dangling pointer to .rodata.
 */
template<fixed_string S, typename T> // The T parameter exists solely for enabling lookup of fixed_string operators (ADL).
class constexpr_string {
public:
    static constexpr auto value = S;

    constexpr operator T() const { return value; }

    // types
    using value_type      = typename T::value_type;
    using pointer         = value_type*;
    using const_pointer   = const value_type*;
    using reference       = value_type&;
    using const_reference = const value_type&;
    using size_type       = size_t;
    using difference_type = std::ptrdiff_t;

    // capacity
    static constexpr auto size = S.size;

    static constexpr auto length = S.length;

    static constexpr auto max_size = S.max_size;

    [[nodiscard]] static constexpr bool empty() noexcept { return S.empty(); }

    // element access
    [[nodiscard]] consteval const_reference operator[](size_type pos) const { return S[pos]; }

    [[nodiscard]] consteval const_reference front() const { return S.front(); }

    [[nodiscard]] consteval const_reference back() const { return S.back(); }

    // string operations
    [[nodiscard]] consteval const_pointer c_str() const noexcept { return S.c_str(); }

    [[nodiscard]] consteval const_pointer data() const noexcept { return S.data(); }

    [[nodiscard]] consteval std::string_view view() const noexcept { return S.view(); }

    consteval operator std::string_view() const noexcept { return S.view(); }

    consteval operator std::string() const noexcept { return static_cast<std::string>(S); }

    template<fixed_string S2>
    consteval friend constexpr_string<S + S2> operator+(constexpr_string, constexpr_string<S2>) noexcept {
        return {};
    }

    template<typename rhs>
    requires std::same_as<decltype(rhs::value), const value_type>
    consteval friend constexpr_string<S + rhs::value> operator+(constexpr_string, rhs) noexcept {
        return {};
    }

    template<typename lhs>
    requires std::same_as<decltype(lhs::value), const value_type>
    consteval friend constexpr_string<lhs::value + S> operator+(lhs, constexpr_string) noexcept {
        return {};
    }

    template<typename c>
    requires std::same_as<decltype(c::value), const value_type>
    consteval std::integral_constant<size_t, S.find_char(c::value)> find_char(c) const noexcept {
        return {};
    }

    template<typename NewSize>
    consteval constexpr_string<resize(S, NewSize())> resize(NewSize) const noexcept {
        return {};
    }

    template<typename Offset, typename NewSize = std::integral_constant<size_t, size - Offset::value>>
    consteval constexpr_string<substring(S, Offset(), NewSize())> substring(Offset, NewSize = {}) const noexcept {
        return {};
    }
};

template<fixed_string S1, typename T1, fixed_string S2, typename T2>
consteval bool operator==(constexpr_string<S1, T1>, constexpr_string<S2, T2>) {
    return S1 == S2;
}

namespace detail {
template<std::integral auto N>
consteval auto fixed_string_from_number_impl() {
    constexpr size_t buf_len = [] {
        auto   x   = N;
        size_t len = x < 0 ? 1u : 0u; // minus character
        while (x != 0) {              // count digits
            ++len;
            x /= 10;
        }
        return len;
    }();
    fixed_string<buf_len> ret{};

    constexpr bool negative = N < 0;

    // do *not* do abs(N) here to support INT_MIN
    auto   x = N;
    size_t i = buf_len;
    while (x != 0) {
        ret[--i] = static_cast<char>('0' + (negative ? -1 : 1) * static_cast<int>(x % 10));
        x /= 10;
    }
    if (negative) {
        ret[--i] = '-';
    }
    if (i != 0) { // this should be impossible
        throw i;
    }
    return ret;
}

template<typename T>
[[nodiscard]] std::string local_type_name() noexcept {
    std::string type_name = typeid(T).name();
    int         status;
    char*       demangled_name = abi::__cxa_demangle(type_name.c_str(), nullptr, nullptr, &status);
    if (status == 0) {
        std::string ret(demangled_name);
        free(demangled_name);
        return ret;
    } else {
        free(demangled_name);
        return typeid(T).name();
    }
}

inline std::string makePortableTypeName(std::string_view name) {
    auto trimmed = [](std::string_view view) {
        while (view.front() == ' ') {
            view.remove_prefix(1);
        }
        while (view.back() == ' ') {
            view.remove_suffix(1);
        }
        return view;
    };

    using namespace std::string_literals;
    using gr::meta::detail::local_type_name;
    static const auto typeMapping = std::array<std::pair<std::string, std::string>, 17>{{
        {local_type_name<std::int8_t>(), "int8"s}, {local_type_name<std::int16_t>(), "int16"s}, {local_type_name<std::int32_t>(), "int32"s}, {local_type_name<std::int64_t>(), "int64"s},         //
        {local_type_name<std::uint8_t>(), "uint8"s}, {local_type_name<std::uint16_t>(), "uint16"s}, {local_type_name<std::uint32_t>(), "uint32"s}, {local_type_name<std::uint64_t>(), "uint64"s}, //
        {local_type_name<float>(), "float32"s}, {local_type_name<double>(), "float64"},                                                                                                           //                                                                                                                                                                                                                                                        //
        {local_type_name<std::float32_t>(), "float32"s}, {local_type_name<std::float64_t>(), "float64"},                                                                                          //
        {local_type_name<std::string>(), "string"s},                                                                                                                                              //
        {local_type_name<std::complex<float>>(), "complex<float32>"s}, {local_type_name<std::complex<double>>(), "complex<float64>"s},                                                            //
        {local_type_name<std::complex<std::float32_t>>(), "complex<float32>"s}, {local_type_name<std::complex<std::float64_t>>(), "complex<float64>"s},                                           //
    }};

    const auto it = std::ranges::find_if(typeMapping, [&](const auto& pair) { return pair.first == name; });
    if (it != typeMapping.end()) {
        return it->second;
    }

    auto stripStdPrivates = [](std::string_view _name) -> std::string {
        // There's an issue in std::regex in libstdcpp which tries to construct
        // a vector of larger-than-possible size in some cases. Need to
        // implement this manually. To simplify, we will remove any namespace
        // starting with an underscore.
        // static const std::regex stdPrivate("::_[A-Z_][a-zA-Z0-9_]*");
        // return std::regex_replace(std::string(_name), stdPrivate, std::string());
        std::string result(_name);
        std::size_t oldStart = 0UZ;
        while (true) {
            auto delStart = result.find("::_"s, oldStart);
            if (delStart == std::string::npos) {
                break;
            }

            auto delEnd = delStart + 3;
            while (delEnd < result.size() && (std::isalnum(result[delEnd]) || result[delEnd] == '_')) {
                delEnd++;
            }

            result.erase(delStart, delEnd - delStart);
            oldStart = delStart;
        }
        return result;
    };

    std::string_view view   = name;
    auto             cursor = view.find("<");
    if (cursor == std::string_view::npos) {
        return stripStdPrivates(std::string{name});
    }
    auto base = view.substr(0, cursor);

    view.remove_prefix(cursor + 1);
    if (!view.ends_with(">")) {
        return stripStdPrivates(std::string{name});
    }
    view.remove_suffix(1);
    while (view.back() == ' ') {
        view.remove_suffix(1);
    }

    std::vector<std::string> params;

    std::size_t depth = 0;
    cursor            = 0;

    while (cursor < view.size()) {
        if (view[cursor] == '<') {
            depth++;
        } else if (view[cursor] == '>') {
            depth--;
        } else if (view[cursor] == ',' && depth == 0) {
            auto param = trimmed(view.substr(0, cursor));
            params.push_back(makePortableTypeName(param));
            view.remove_prefix(cursor + 1);
            cursor = 0;
            continue;
        }
        cursor++;
    }
    params.push_back(makePortableTypeName(trimmed(view)));
    auto join = [](const auto& range, std::string_view sep = ", ") -> std::string {
        std::string out;
        auto        it2  = std::ranges::begin(range);
        const auto  end2 = std::ranges::end(range);
        if (it2 != end2) {
            out += std::format("{}", *it2);
            while (++it2 != end2) {
                out += std::format("{}{}", sep, *it2);
            }
        }
        return out;
    };
    return std::format("{}<{}>", base, join(params, ", "));
}

} // namespace detail

template<std::integral auto N>
inline constexpr auto fixed_string_from_number = detail::fixed_string_from_number_impl<N>();

template<std::integral auto N>
requires(N >= 0 and N < 10)
inline constexpr auto fixed_string_from_number<N> = fixed_string<1>('0' + N);

template<std::integral auto N>
using constexpr_string_from_number_t = constexpr_string<fixed_string_from_number<N>>;

template<std::integral auto N>
inline constexpr constexpr_string_from_number_t<N> constexpr_string_from_number_v{};

template<int N>
[[deprecated("use fixed_string_from_number<N> or constexpr_string_from_number_v<N> instead")]]
constexpr auto make_fixed_string() noexcept {
    return fixed_string_from_number<N>;
}

static_assert(fixed_string("0") == fixed_string_from_number<0>);
static_assert(fixed_string("1") == fixed_string_from_number<1>);
static_assert(fixed_string("-1") == fixed_string_from_number<-1>);
static_assert(fixed_string("2") == fixed_string_from_number<2>);
static_assert(fixed_string("123") == fixed_string_from_number<123>);
static_assert((fixed_string("out") + fixed_string_from_number<123>) == fixed_string("out123"));

template<typename T>
[[nodiscard]] std::string type_name() noexcept {
    return detail::makePortableTypeName(detail::local_type_name<T>());
}

inline std::string shorten_type_name(std::string_view name) {
    using namespace std::string_view_literals;

    const bool hasLeading   = name.starts_with("::"sv);
    const bool hasTrailing  = name.ends_with("::"sv);
    const auto toStringView = [](auto&& r) { return std::string_view(&*r.begin(), static_cast<std::size_t>(std::ranges::distance(r))); };

    std::vector<std::string_view> parts;
    for (auto&& token : name | std::views::split("::"sv)) {
        if (auto sv = toStringView(token); !sv.empty()) {
            parts.push_back(sv);
        }
    }

    if (parts.empty()) {
        return hasLeading || hasTrailing ? "::" : "";
    }

    std::string result;
    if (hasLeading) {
        result += "::"sv;
    }

    if (parts.size() == 1UZ) {
        result += hasTrailing ? std::string(1, parts.front().front()) : std::string(parts.front());
    } else {
        for (auto&& part : parts | std::views::take(parts.size() - 1UZ)) {
            result += part.front();
        }
        if (!hasTrailing) {
            result += "::";
        }
        result += parts.back();
    }

    if (hasTrailing) {
        result += "::";
    }

    return result;
}

template<fixed_string val>
struct message_type {};

template<class... T>
constexpr bool always_false = false;

constexpr std::size_t invalid_index              = std::numeric_limits<std::size_t>::max();
constexpr std::size_t default_message_port_index = std::numeric_limits<std::size_t>::max() - 1UZ;

/**
 * T is tuple-like if it implements std::tuple_size, std::tuple_element, and std::get.
 * Tuples with size 0 are excluded.
 */
template<typename T>
concept tuple_like = (std::tuple_size<T>::value > 0) && requires(T tup) {
    { std::get<0>(tup) } -> std::same_as<typename std::tuple_element_t<0, T>&>;
};

template<template<typename...> class Template, typename Class>
struct is_instantiation : std::false_type {};

template<template<typename...> class Template, typename... Args>
struct is_instantiation<Template, Template<Args...>> : std::true_type {};
template<typename Class, template<typename...> class Template>
concept is_instantiation_of = is_instantiation<Template, Class>::value;

template<typename T>
concept map_type = is_instantiation_of<T, std::map> || is_instantiation_of<T, std::unordered_map>;

template<typename T>
concept vector_type = is_instantiation_of<std::remove_cv_t<T>, std::vector>;

template<typename T>
struct is_std_array_type : std::false_type {};

template<typename T, std::size_t N>
struct is_std_array_type<std::array<T, N>> : std::true_type {};

template<typename T>
concept array_type = is_std_array_type<std::remove_cv_t<T>>::value;

template<typename T, typename V = void>
concept array_or_vector_type = (vector_type<T> || array_type<T>) && (std::same_as<V, void> || std::same_as<typename T::value_type, V>);

namespace stdx = vir::stdx;

template<typename V, typename T = void>
concept any_simd = stdx::is_simd_v<V> && (std::same_as<T, void> || std::same_as<T, typename V::value_type>);

template<typename V, typename T>
concept t_or_simd = std::same_as<V, T> || any_simd<V, T>;

template<typename T>
concept complex_like = std::is_same_v<T, std::complex<float>> || std::is_same_v<T, std::complex<double>>;

template<fixed_string Name, typename PortList>
consteval std::size_t indexForName() {
    auto helper = []<std::size_t... Ids>(std::index_sequence<Ids...>) {
        auto static_name_for_index = [](auto id) {
            using Port = typename PortList::template at<id>;
            if constexpr (requires { Port::Name; }) {
                return Port::Name;
            } else {
                // should never see a tuple here => needs to be flattened into given PortList earlier
                return Port::value_type::Name;
            }
        };

        constexpr int n_matches = ((static_name_for_index(std::integral_constant<size_t, Ids>()) == Name) + ...);
        static_assert(n_matches <= 1, "Multiple ports with that name were found. The name must be unique. You can "
                                      "still use a port index instead.");
        static_assert(n_matches == 1, "No port with the given name exists.");
        constexpr std::size_t result = (((static_name_for_index(std::integral_constant<size_t, Ids>()) == Name) * Ids) + ...);
        return result;
    };
    return helper(std::make_index_sequence<PortList::size>());
}

// template<template<typename...> typename Type, typename... Items>
// using find_type = decltype(std::tuple_cat(std::declval<std::conditional_t<is_instantiation_of<Items, Type>, std::tuple<Items>, std::tuple<>>>()...));

template<template<typename> typename Pred, typename... Items>
struct find_type;

template<template<typename> typename Pred>
struct find_type<Pred> {
    using type = std::tuple<>;
};

template<template<typename> typename Pred, typename First, typename... Rest>
struct find_type<Pred, First, Rest...> {
    using type = decltype(std::tuple_cat(std::conditional_t<Pred<First>::value, std::tuple<First>, std::tuple<>>(), typename find_type<Pred, Rest...>::type()));
};

template<template<typename> typename Pred, typename... Items>
using find_type_t = typename find_type<Pred, Items...>::type;

template<typename Tuple, typename Default = void>
struct get_first_or_default;

template<typename First, typename... Rest, typename Default>
struct get_first_or_default<std::tuple<First, Rest...>, Default> {
    using type = First;
};

template<typename Default>
struct get_first_or_default<std::tuple<>, Default> {
    using type = Default;
};

template<typename Tuple, typename Default = void>
using get_first_or_default_t = typename get_first_or_default<Tuple, Default>::type;

template<typename... Lambdas>
struct overloaded : Lambdas... {
    using Lambdas::operator()...;
};

template<typename... Lambdas>
overloaded(Lambdas...) -> overloaded<Lambdas...>;

namespace detail {
template<template<typename...> typename Mapper, template<typename...> typename Wrapper, typename... Args>
Wrapper<Mapper<Args>...>* type_transform_impl(Wrapper<Args...>*);

template<template<typename...> typename Mapper, typename T>
Mapper<T>* type_transform_impl(T*);

template<template<typename...> typename Mapper>
void* type_transform_impl(void*);
} // namespace detail

template<template<typename...> typename Mapper, typename T>
using type_transform = std::remove_pointer_t<decltype(detail::type_transform_impl<Mapper>(static_cast<T*>(nullptr)))>;

template<typename Arg, typename... Args>
auto safe_min(Arg&& arg, Args&&... args) {
    if constexpr (sizeof...(Args) == 0) {
        return arg;
    } else {
        return std::min(std::forward<Arg>(arg), std::forward<Args>(args)...);
    }
}

template<typename Arg, typename... Args>
auto safe_pair_min(Arg&& arg, Args&&... args) {
    if constexpr (sizeof...(Args) == 0) {
        return arg;
    } else {
        return std::make_pair(std::min(std::forward<Arg>(arg).first, std::forward<Args>(args).first...), std::min(std::forward<Arg>(arg).second, std::forward<Args>(args).second...));
    }
}

template<typename Function, typename Tuple, typename... Tuples>
auto tuple_for_each(Function&& function, Tuple&& tuple, Tuples&&... tuples) {
    static_assert(((std::tuple_size_v<std::remove_cvref_t<Tuple>> == std::tuple_size_v<std::remove_cvref_t<Tuples>>) && ...));
    return [&]<std::size_t... Idx>(std::index_sequence<Idx...>) { (([&function, &tuple, &tuples...](auto I) { function(std::get<I>(tuple), std::get<I>(tuples)...); }(std::integral_constant<std::size_t, Idx>{}), ...)); }(std::make_index_sequence<std::tuple_size_v<std::remove_cvref_t<Tuple>>>());
}

template<typename Function, typename Tuple, typename... Tuples>
void tuple_for_each_enumerate(Function&& function, Tuple&& tuple, Tuples&&... tuples) {
    static_assert(((std::tuple_size_v<std::remove_cvref_t<Tuple>> == std::tuple_size_v<std::remove_cvref_t<Tuples>>) && ...));
    [&]<std::size_t... Idx>(std::index_sequence<Idx...>) { ([&function](auto I, auto&& t0, auto&&... ts) { function(I, std::get<I>(t0), std::get<I>(ts)...); }(std::integral_constant<std::size_t, Idx>{}, tuple, tuples...), ...); }(std::make_index_sequence<std::tuple_size_v<std::remove_cvref_t<Tuple>>>());
}

template<typename Function, typename Tuple, typename... Tuples>
auto tuple_transform(Function&& function, Tuple&& tuple, Tuples&&... tuples) {
    static_assert(((std::tuple_size_v<std::remove_cvref_t<Tuple>> == std::tuple_size_v<std::remove_cvref_t<Tuples>>) && ...));
    return [&]<std::size_t... Idx>(std::index_sequence<Idx...>) { return std::make_tuple([&function, &tuple, &tuples...](auto I) { return function(std::get<I>(tuple), std::get<I>(tuples)...); }(std::integral_constant<std::size_t, Idx>{})...); }(std::make_index_sequence<std::tuple_size_v<std::remove_cvref_t<Tuple>>>());
}

template<typename Function, typename Tuple, typename... Tuples>
auto tuple_transform_enumerated(Function&& function, Tuple&& tuple, Tuples&&... tuples) {
    static_assert(((std::tuple_size_v<std::remove_cvref_t<Tuple>> == std::tuple_size_v<std::remove_cvref_t<Tuples>>) && ...));
    return [&]<std::size_t... Idx>(std::index_sequence<Idx...>) { return std::make_tuple([&function, &tuple, &tuples...](auto I) { return function(I, std::get<I>(tuple), std::get<I>(tuples)...); }(std::integral_constant<std::size_t, Idx>{})...); }(std::make_index_sequence<std::tuple_size_v<std::remove_cvref_t<Tuple>>>());
}

static_assert(std::is_same_v<std::vector<int>, type_transform<std::vector, int>>);
static_assert(std::is_same_v<std::tuple<std::vector<int>, std::vector<float>>, type_transform<std::vector, std::tuple<int, float>>>);
static_assert(std::is_same_v<void, type_transform<std::vector, void>>);

#ifdef __cpp_lib_hardware_interference_size
static inline constexpr const std::size_t kCacheLine = std::hardware_destructive_interference_size;
#else
static inline constexpr const std::size_t kCacheLine = 64;
#endif

namespace detail {

template<typename T>
concept HasValueType = requires { typename T::value_type; };

template<typename T, typename = void>
struct fundamental_base_value_type {
    using type = T;
};

template<HasValueType T>
struct fundamental_base_value_type<T> {
    using type = typename fundamental_base_value_type<typename T::value_type>::type;
};

} // namespace detail

template<typename T>
using fundamental_base_value_type_t = typename detail::fundamental_base_value_type<T>::type;

static_assert(std::is_same_v<fundamental_base_value_type_t<int>, int>);
static_assert(std::is_same_v<fundamental_base_value_type_t<std::vector<float>>, float>);
static_assert(std::is_same_v<fundamental_base_value_type_t<std::vector<std::complex<double>>>, double>);

template<typename T>
concept string_like = std::is_same_v<T, std::string> || std::is_same_v<T, std::string_view> || std::is_convertible_v<T, std::string_view>;

namespace detail {
template<typename T>
struct is_const_member_function : std::false_type {};

template<typename T, typename TReturn, typename... Args>
struct is_const_member_function<TReturn (T::*)(Args...) const> : std::true_type {};

template<typename T, typename TReturn, typename... Args>
struct is_const_member_function<TReturn (T::*)(Args...) const noexcept> : std::true_type {};

template<typename T>
struct is_noexcept_member_function : std::false_type {};

template<typename T, typename TReturn, typename... Args>
struct is_noexcept_member_function<TReturn (T::*)(Args...) noexcept> : std::true_type {};

template<typename T, typename TReturn, typename... Args>
struct is_noexcept_member_function<TReturn (T::*)(Args...) const noexcept> : std::true_type {};
} // namespace detail

template<typename T>
concept IsConstMemberFunction = std::is_member_function_pointer_v<T> && detail::is_const_member_function<T>::value;

template<typename T>
concept IsNoexceptMemberFunction = std::is_member_function_pointer_v<T> && detail::is_noexcept_member_function<T>::value;

} // namespace meta

template<typename Fn>
struct on_scope_exit {
    Fn function;
    on_scope_exit(Fn fn) : function(std::move(fn)) {}
    ~on_scope_exit() { function(); }
};

struct normalise_t {};
inline constexpr normalise_t normalise{};

struct Ratio {
    // data first
    std::int32_t numerator{1};
    std::int32_t denominator{1};

    static constexpr Ratio invalid() noexcept { return Ratio{0, 0}; }

    constexpr Ratio(std::int32_t n = 1, std::int32_t d = 1) noexcept : numerator(n), denominator(d) {}
    constexpr Ratio(std::int32_t n, std::int32_t d, normalise_t) noexcept : Ratio(n, d) { normalise(); }

    explicit constexpr Ratio(std::string_view sv) noexcept : Ratio(parse(sv).value_or(invalid())) {}
    explicit constexpr Ratio(std::string_view sv, normalise_t) noexcept : Ratio(sv) { normalise(); }

    template<class R> // std::ratio<N,D>
    constexpr static Ratio from() {
        return Ratio(R::num, R::den);
    }

    constexpr std::int32_t num() const noexcept { return numerator; }
    constexpr std::int32_t den() const noexcept { return denominator; }

    template<typename T = double>
    constexpr T value() const noexcept {
        return static_cast<T>(numerator) / static_cast<T>(denominator);
    }

    constexpr Ratio reciprocal() const {
        assert(numerator != 0);
        return Ratio(denominator, numerator);
    }

    friend constexpr Ratio operator+(const Ratio& a, const Ratio& b) {
        // minimise overflow: a/b + c/d = (a*lcm_den/b)*lcm_den + ...
        auto g = std::gcd(a.denominator, b.denominator);
        // a/b + c/d = (a*(d/g) + c*(b/g)) / lcm
        std::int32_t l = b.denominator / g;
        std::int32_t r = a.denominator / g;
        return Ratio(a.numerator * l + b.numerator * r, a.denominator * l);
    }

    friend constexpr Ratio operator-(const Ratio& a, const Ratio& b) {
        auto         g = std::gcd(a.denominator, b.denominator);
        std::int32_t l = b.denominator / g;
        std::int32_t r = a.denominator / g;
        return Ratio(a.numerator * l - b.numerator * r, a.denominator * l);
    }

    friend constexpr Ratio operator*(const Ratio& a, const Ratio& b) {
        auto g1 = std::gcd(a.numerator, b.denominator);
        auto g2 = std::gcd(b.numerator, a.denominator);
        return Ratio((a.numerator / g1) * (b.numerator / g2), (a.denominator / g2) * (b.denominator / g1));
    }

    friend constexpr Ratio operator/(const Ratio& a, const Ratio& b) {
        assert(b.numerator != 0);
        // a/b ÷ c/d = (a*d)/(b*c)
        auto g1 = std::gcd(a.numerator, b.numerator);
        auto g2 = std::gcd(a.denominator, b.denominator);
        return Ratio((a.numerator / g1) * (b.denominator / g2), (a.denominator / g2) * (b.numerator / g1));
    }

    friend constexpr Ratio operator-(const Ratio& r) { return Ratio(-r.numerator, r.denominator); }
    friend constexpr auto  operator<=>(const Ratio& a, const Ratio& b) = default;
    friend constexpr bool  operator==(const Ratio& a, const Ratio& b)  = default;

    constexpr Ratio& operator+=(const Ratio& other) { return *this = *this + other; }
    constexpr Ratio& operator-=(const Ratio& other) { return *this = *this - other; }
    constexpr Ratio& operator*=(const Ratio& other) { return *this = *this * other; }
    constexpr Ratio& operator/=(const Ratio& other) { return *this = *this / other; }

    // helper
    static constexpr std::optional<Ratio> parse(std::string_view sv) noexcept {
        constexpr auto parse = [](std::string_view s) -> std::optional<std::int32_t> {
            if (s.empty()) {
                return std::nullopt;
            }

            bool        neg = (s.front() == '-');
            std::size_t i   = (s.front() == '+' || neg) ? 1 : 0;
            if (i == s.size()) {
                return std::nullopt;
            }

            std::int32_t   v = 0;
            constexpr auto M = std::numeric_limits<std::int32_t>::max();
            constexpr auto m = std::numeric_limits<std::int32_t>::min();

            for (; i < s.size(); ++i) {
                char c = s[i];
                if (c < '0' || c > '9') {
                    return std::nullopt;
                }
                const std::int32_t d = c - '0';

                if (!neg) {
                    if (v > (M - d) / 10) {
                        return std::nullopt;
                    }
                    v = v * 10 + d;
                } else {
                    if (v < (m + d) / 10) {
                        return std::nullopt;
                    }
                    v = v * 10 - d;
                }
            }
            return v;
        };

        const auto slash = sv.find('/');
        const auto lhs   = sv.substr(0, slash);
        const auto rhs   = (slash == std::string_view::npos) ? std::string_view{"1"} : sv.substr(slash + 1);

        auto n = parse(lhs);
        auto d = parse(rhs);
        if (!n || !d || *d == 0) {
            return std::nullopt;
        }
        return Ratio{*n, *d};
    }

    constexpr void normalise() {
        if (denominator == 0) {
            std::unreachable();
        }
        if (denominator < 0) {
            denominator = -denominator;
            numerator   = -numerator;
        }
        if (auto g = std::gcd(numerator, denominator); g > 1) {
            numerator /= g;
            denominator /= g;
        }
    }
};

} // namespace gr

template<>
struct std::formatter<gr::Ratio> : std::formatter<std::string_view> {
    template<class FormatContext>
    auto format(const gr::Ratio& r, FormatContext& ctx) const {
        return std::formatter<std::string_view>::format((r.den() == 1) ? std::format("{}", r.num()) : std::format("{}/{}", r.num(), r.den()), ctx);
    }
};

#endif // include guard
