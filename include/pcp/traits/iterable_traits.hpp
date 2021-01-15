#ifndef PCP_TRAITS_ITERABLE_TRAITS_HPP
#define PCP_TRAITS_ITERABLE_TRAITS_HPP

/**
 * @file
 * @ingroup traits
 */

#include <type_traits>

namespace pcp {
namespace traits {

/**
 * @ingroup traits
 * @brief
 * Iterable concept requires an Iterable type to have begin()/end() pair.
 * @tparam Iterable
 */
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
    using value_type    = typename std::iterator_traits<iterator_type>::value_type;
};

/**
 * @ingroup traits
 * @brief
 * Compile-time check for Iterable concept
 * @tparam Iterable
 */
template <class Iterable>
static constexpr bool is_iterable_v = iterable<Iterable>::value;

/**
 * @ingroup traits
 * @brief
 * Compile-time query for the underlying iterated over type of the Iterable type
 * @tparam Iterable
 */
template <class Iterable>
using value_type_of_iterable_t = typename iterable<Iterable>::value_type;

} // namespace traits
} // namespace pcp

#endif // PCP_TRAITS_ITERABLE_TRAITS_HPP