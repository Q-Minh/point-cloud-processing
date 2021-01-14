#ifndef PCP_TRAITS_ITERABLE_TRAITS_HPP
#define PCP_TRAITS_ITERABLE_TRAITS_HPP

#include <type_traits>

namespace pcp {
namespace traits {

template <class Iterable, class = void>
struct iterable : std::false_type
{
};

template <class Iterable>
struct iterable<
    Iterable,
    std::void_t<
        decltype(std::declval<Iterable&>().begin()),
        decltype(std::declval<Iterable&>().end())>> : std::true_type
{
    using iterator_type = decltype(std::declval<Iterable&>().begin());
    using value_type = typename std::iterator_traits<iterator_type>::value_type;
};

template <class Iterable>
static constexpr bool is_iterable_v = iterable<Iterable>::value;

template <class Iterable>
using value_type_of_iterable_t = typename iterable<Iterable>::value_type;

} // namespace traits
} // namespace pcp

#endif // PCP_TRAITS_ITERABLE_TRAITS_HPP