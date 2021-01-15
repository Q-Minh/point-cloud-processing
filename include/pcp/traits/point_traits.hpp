#ifndef PCP_TRAITS_POINT_TRAITS_HPP
#define PCP_TRAITS_POINT_TRAITS_HPP

/**
 * @file
 * @ingroup traits
 */

#include <type_traits>

namespace pcp {
namespace traits {

template <class PointView, class = void>
struct is_point_view : std::false_type
{
};

/**
 * @ingroup traits
 * @brief
 * PointView requirements:
 * - coordinate_type type member
 * - copy constructible
 * - copy assignable
 * - move constructible
 * - move assignable
 * - x,y,z getters
 * - x,y,z setters
 * - equality/inequality operator
 * @tparam PointView Type to inspect
 */
template <class PointView>
struct is_point_view<
    PointView,
    std::void_t<
        typename PointView::coordinate_type,
        decltype(std::declval<PointView&>().x()),
        decltype(std::declval<PointView&>().y()),
        decltype(std::declval<PointView&>().z()),
        decltype(std::declval<PointView&>().x(std::declval<typename PointView::coordinate_type>())),
        decltype(std::declval<PointView&>().y(std::declval<typename PointView::coordinate_type>())),
        decltype(std::declval<PointView&>().z(
            std::declval<typename PointView::coordinate_type>()))>> : std::true_type
{
    static_assert(std::is_copy_constructible_v<PointView>, "PointView must be copy constructible");
    static_assert(std::is_copy_assignable_v<PointView>, "PointView must be copy assignable");
};

/**
 * @ingroup traits
 * @brief
 * Compile-time check for PointView concept
 * @tparam PointView
 */
template <class PointView>
static constexpr bool is_point_view_v = is_point_view<PointView>::value;

template <class Point, class = void>
struct is_point : std::false_type
{
};

/**
 * @ingroup traits
 * @brief
 * Point requirements (refines PointView):
 * - default constructible
 * - parameter constructor from x,y,z
 * - *,/,+,- operators
 */
template <class Point>
struct is_point<
    Point,
    std::void_t<
        decltype(std::declval<typename Point::coordinate_type>() * std::declval<Point&>()),
        decltype(std::declval<Point&>() / std::declval<typename Point::coordinate_type>()),
        decltype(std::declval<Point&>() + std::declval<Point&>()),
        decltype(std::declval<Point&>() - std::declval<Point&>()),
        decltype(-std::declval<Point&>())>> : is_point_view<Point>
{
    static_assert(std::is_default_constructible_v<Point>, "Point must be default constructible");
    static_assert(
        std::is_constructible_v<
            Point,
            typename Point::coordinate_type,
            typename Point::coordinate_type,
            typename Point::coordinate_type>,
        "Point must be constructible from (x,y,z) coordinates");
};

/**
 * @ingroup traits
 * @brief
 * Compile-time check for Point concept
 * @tparam Point
 */
template <class Point>
static constexpr bool is_point_v = is_point<Point>::value;

template <class PointView1, class PointView2, class = void>
struct is_point_view_equality_comparable_to : std::false_type
{
};

template <class PointView1, class PointView2>
struct is_point_view_equality_comparable_to<
    PointView1,
    PointView2,
    std::void_t<
        decltype(std::declval<PointView1&>() == std::declval<PointView2&>()),
        decltype(std::declval<PointView1&>() != std::declval<PointView2&>())>> : std::true_type
{
};

/**
 * @ingroup traits
 * @brief
 * Compile-time check to verify if PointView1 is equality/inequality comparable to PointView2
 * @tparam PointView1
 * @tparam PointView2
 */
template <class PointView1, class PointView2>
static constexpr bool is_point_view_equality_comparable_to_v =
    is_point_view_equality_comparable_to<PointView1, PointView2>::value;

template <class PointView1, class PointView2, class = void>
struct is_point_view_assignable_from : std::false_type
{
};

template <class PointView1, class PointView2>
struct is_point_view_assignable_from<
    PointView1,
    PointView2,
    std::void_t<decltype(std::declval<PointView1&>() = std::declval<PointView2&>())>>
    : std::true_type
{
};

/**
 * @ingroup traits
 * @brief
 * Compile-time check to verify is PointView1 is assignable from PointView2
 * @tparam PointView1
 * @tparam PointView2
 */
template <class PointView1, class PointView2>
static constexpr bool is_point_view_assignable_from_v =
    is_point_view_assignable_from<PointView1, PointView2>::value;

} // namespace traits
} // namespace pcp

#endif // PCP_TRAITS_POINT_TRAITS_HPP