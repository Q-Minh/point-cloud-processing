#ifndef PCP_TRAITS_KNN_SEARCH_TRAITS_HPP
#define PCP_TRAITS_KNN_SEARCH_TRAITS_HPP

/**
 * @file
 * @ingroup traits
 */

#include "iterable_traits.hpp"
#include "point_traits.hpp"

#include <type_traits>

namespace pcp {
namespace traits {

template <class KnnSearcher, class Element, class = void>
struct is_knn_searcher : std::false_type
{
};

/**
 * @ingroup traits
 * @brief
 * KnnSearcher concept
 * Terminology:
 * - knn is a KnnSearcher instance
 * - V is the input element's type
 * - v is an input element
 * - R is the return type of the KnnSearcher
 * - r is a return value of a call to knn(v)
 * Requirements:
 * - r is iterable (has a begin(),end())
 * - auto r = knn(v); is supported
 * @tparam KnnSearcher Type to test for the KnnSearcher concept
 * @tparam Element Type of input element called by the KnnSearcher
 */
template <class KnnSearcher, class Element>
struct is_knn_searcher<
    KnnSearcher,
    Element,
    std::void_t<decltype(std::declval<KnnSearcher&>()(std::declval<Element&>()))>> : std::true_type
{
    using result_type = std::invoke_result_t<KnnSearcher, Element>;
    static_assert(
        traits::is_iterable_v<result_type>,
        "Return type of KnnSearcher must be iterable");
};

/**
 * @ingroup traits
 * @brief
 * Compile-time check for KnnSearcher concept
 * @tparam KnnSearcher
 * @tparam Element
 */
template <class KnnSearcher, class Element>
static constexpr bool is_knn_searcher_v = is_knn_searcher<KnnSearcher, Element>::value;

} // namespace traits
} // namespace pcp

#endif // PCP_TRAITS_KNN_SEARCH_TRAITS_HPP