#pragma once

#include <type_traits>

namespace pcp {
namespace traits {

template <class Iterable, class Element, class = void>
struct is_iterable : std::false_type
{
};

template <class Iterable, class Element>
struct is_iterable<
    Iterable,
    Element,
    std::void_t<
        decltype(std::declval<Iterable&>().begin()),
        decltype(std::declval<Iterable&>().end())>> : std::true_type
{
    static_assert(
        std::is_convertible_v<decltype(*std::declval<Iterable&>().begin()), Element>,
        "Iterable does not provide iterators to Element");
};

template <class Iterable, class Element>
static constexpr bool is_iterable_v = is_iterable<Iterable, Element>::value;

} // namespace traits
} // namespace pcp