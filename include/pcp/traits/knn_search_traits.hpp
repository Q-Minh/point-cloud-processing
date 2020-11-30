#pragma once

#include "iterable_traits.hpp"

#include <type_traits>

namespace pcp {
namespace traits {

template <class KnnSearcher, class Element, class KType, class = void>
struct is_knn_searcher : std::false_type
{
};

template <class KnnSearcher, class Element, class KType>
struct is_knn_searcher<
    KnnSearcher,
    Element,
    KType,
    std::void_t<decltype(
        std::declval<KnnSearcher&>()(std::declval<Element&>(), std::declval<KType>()))>>
    : std::true_type
{
    static_assert(std::is_integral_v<KType>, "KType must be an integral type");
    static_assert(
        is_iterable_v<std::invoke_result_t<KnnSearcher, Element, KType>, Element>,
        "Return type of KnnSearcher must be satisfy Iterable concept");
};

template <class KnnSearcher, class Element, class KType>
static constexpr bool is_knn_searcher_v = is_knn_searcher<KnnSearcher, Element, KType>::value;

} // namespace traits
} // namespace pcp