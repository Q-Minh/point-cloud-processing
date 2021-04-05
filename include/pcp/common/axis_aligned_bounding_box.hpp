#ifndef PCP_COMMON_AXIS_ALIGNED_BOUNDING_BOX_HPP
#define PCP_COMMON_AXIS_ALIGNED_BOUNDING_BOX_HPP

/**
 * @file
 * @ingroup common
 */

#include "pcp/traits/point_traits.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <execution>
#include <limits>
#include <mutex>
#include <numeric>
#include <range/v3/view/zip.hpp>
#include <shared_mutex>
#include <tuple>

namespace pcp {

/**
 * @ingroup geometric-primitives
 * @brief
 * https://en.wikipedia.org/wiki/Bounding_volume
 * @tparam CoordinateType
 * @tparam K The dimensions
 */
template <class CoordinateType, std::size_t K>
struct kd_axis_aligned_bounding_box_t
{
    using scalar_type = CoordinateType;
    using point_type  = std::array<scalar_type, K>;

    point_type min{}, max{};

    /**
     * @brief
     * Containment predicate.
     * @param p
     * @return true if point p is contained in this AABB
     */
    bool contains(point_type const& p) const
    {
        auto const greater_than_or_equal = [](point_type const& p1, point_type const& p2) -> bool {
            auto rng = ranges::views::zip(p1, p2);
            bool const is_greater_than_or_equal =
                std::all_of(rng.begin(), rng.end(), [](auto&& tup) {
                    auto const& c1 = std::get<0>(tup);
                    auto const& c2 = std::get<1>(tup);
                    return c1 >= c2;
                });
            return is_greater_than_or_equal;
        };

        /**
         * @brief
         * @param p
         * @return
         */
        auto const less_than_or_equal = [](point_type const& p1, point_type const& p2) -> bool {
            auto rng                         = ranges::views::zip(p1, p2);
            bool const is_less_than_or_equal = std::all_of(rng.begin(), rng.end(), [](auto&& tup) {
                auto const& c1 = std::get<0>(tup);
                auto const& c2 = std::get<1>(tup);
                return c1 <= c2;
            });
            return is_less_than_or_equal;
        };

        return greater_than_or_equal(p, min) && less_than_or_equal(p, max);
    }
    /**
     * Predicate for closest point from this AABB to the point p
     * in Euclidean space.
     * @param p The point closest to the returned point
     * @return The closest point from this AABB to the point p
     */
    point_type nearest_point_from(point_type const& p) const
    {
        point_type nearest_point = p;
        for (auto i = 0u; i < p.size(); ++i)
        {
            nearest_point[i] = std::clamp(nearest_point[i], min[i], max[i]);
        }

        return nearest_point;
    }
};

/**
 * @ingroup geometric-primitives
 * @brief
 * https://en.wikipedia.org/wiki/Bounding_volume
 * @tparam Point Type satisfying Point concept
 */
template <class Point>
struct axis_aligned_bounding_box_t
{
    using point_type = Point;
    static_assert(traits::is_point_v<Point>, "Point must satisfy Point concept");

    Point min{0., 0., 0.}, max{0., 0., 0.};

    /**
     * Containment predicate.
     * @return true if point p is contained in this AABB
     */
    template <class TPoint>
    bool contains(TPoint const& p) const
    {
        static_assert(traits::is_point_view_v<TPoint>, "TPoint must satisfy PointView concept");

        auto const greater_than_or_equal = [](TPoint const& p1, Point const& p2) -> bool {
            return p1.x() >= p2.x() && p1.y() >= p2.y() && p1.z() >= p2.z();
        };

        auto const less_than_or_equal = [](TPoint const& p1, Point const& p2) -> bool {
            return p1.x() <= p2.x() && p1.y() <= p2.y() && p1.z() <= p2.z();
        };

        return greater_than_or_equal(p, min) && less_than_or_equal(p, max);
    }

    /**
     * @return the center point of this AABB
     */
    Point center() const { return (min + max) / 2.f; }

    /**
     * Predicate for closest point from this AABB to the point <code>p</code>
     * in Euclidean space.
     * @param p The point closest to the returned point
     * @return The closest point from this AABB to the point <code>p</code>
     */
    template <class TPoint>
    Point nearest_point_from(TPoint const& p) const
    {
        static_assert(traits::is_point_view_v<TPoint>, "TPoint must satisfy PointView concept");
        Point nearest_point{p.x(), p.y(), p.z()};
        nearest_point.x(std::clamp(nearest_point.x(), min.x(), max.x()));
        nearest_point.y(std::clamp(nearest_point.y(), min.y(), max.y()));
        nearest_point.z(std::clamp(nearest_point.z(), min.z(), max.z()));

        return nearest_point;
    }
};

/**
 * @ingroup common
 * @brief
 * Computes the the k dimensional axis aligned bounding box from a group of points
 * @tparam CoordinateType
 * @tparam K
 * @tparam CoordinateMap
 * @tparam ForwardIter
 * @param begin
 * @param end
 * @param coordinate_map
 * @return
 */
template <class CoordinateType, std::size_t K, class CoordinateMap, class ForwardIter>
inline kd_axis_aligned_bounding_box_t<CoordinateType, K>
kd_bounding_box(ForwardIter begin, ForwardIter end, CoordinateMap const& coordinate_map)
{
    using aabb_type   = kd_axis_aligned_bounding_box_t<CoordinateType, K>;
    using point_type  = typename aabb_type::point_type;
    using scalar_type = typename aabb_type::scalar_type;

    aabb_type aabb;
    for (auto i = 0u; i < aabb.min.size(); ++i)
    {
        aabb.min[i] = std::numeric_limits<scalar_type>::max();
        aabb.max[i] = std::numeric_limits<scalar_type>::lowest();
    }
    std::array<std::shared_mutex, K> max_mutex;
    std::array<std::shared_mutex, K> min_mutex;
    // TODO: Expose sequential overload?
    std::for_each(std::execution::par, begin, end, [&](auto const& element) {
        auto const& p = coordinate_map(element);
        for (auto i = 0u; i < aabb.min.size(); ++i)
        {
            if (std::shared_lock lock_min_read(min_mutex[i]); p[i] < aabb.min[i])
            {
                lock_min_read.unlock();
                if (std::unique_lock lock_min_write(min_mutex[i]); p[i] < aabb.min[i])
                    aabb.min[i] = p[i];
            }

            if (std::shared_lock lock_max_read(max_mutex[i]); p[i] > aabb.max[i])
            {
                lock_max_read.unlock();
                if (std::unique_lock lock_max_write(max_mutex[i]); p[i] > aabb.max[i])
                    aabb.max[i] = p[i];
            }
        }
    });
    return aabb;
}

/**
 * @ingroup common
 * @brief
 * Computes the axis aligned bounding box from a group of points
 * @tparam ForwardIter
 * @tparam Point
 * @tparam AABB
 * @param begin
 * @param end
 * @return
 */
template <class ForwardIter, class Point, class AABB = axis_aligned_bounding_box_t<Point>>
inline AABB bounding_box(ForwardIter begin, ForwardIter end)
{
    using aabb_type       = AABB;
    using point_view_type = typename std::iterator_traits<ForwardIter>::value_type;

    static_assert(traits::is_point_v<Point>, "Point must satisfy Point concept");
    static_assert(
        traits::is_point_view_v<point_view_type>,
        "std::iterator_traits<ForwardIter>::value_type must satisfy PointView concept");

    aabb_type aabb;
    aabb.min.x(std::numeric_limits<typename Point::coordinate_type>::max());
    aabb.min.y(std::numeric_limits<typename Point::coordinate_type>::max());
    aabb.min.z(std::numeric_limits<typename Point::coordinate_type>::max());
    aabb.max.x(std::numeric_limits<typename Point::coordinate_type>::lowest());
    aabb.max.y(std::numeric_limits<typename Point::coordinate_type>::lowest());
    aabb.max.z(std::numeric_limits<typename Point::coordinate_type>::lowest());

    aabb = std::accumulate(begin, end, aabb, [](aabb_type& bbox, point_view_type const& p) {
        if (p.x() < bbox.min.x())
            bbox.min.x(p.x());
        if (p.y() < bbox.min.y())
            bbox.min.y(p.y());
        if (p.z() < bbox.min.z())
            bbox.min.z(p.z());
        if (p.x() > bbox.max.x())
            bbox.max.x(p.x());
        if (p.y() > bbox.max.y())
            bbox.max.y(p.y());
        if (p.z() > bbox.max.z())
            bbox.max.z(p.z());

        return bbox;
    });

    return aabb;
}

template <
    class ForwardIter,
    class PointMap,
    class Point =
        std::invoke_result_t<PointMap, typename std::iterator_traits<ForwardIter>::value_type>,
    class AABB = axis_aligned_bounding_box_t<Point>>
inline AABB bounding_box(ForwardIter begin, ForwardIter end, PointMap const& point_map)
{
    using aabb_type          = AABB;
    using input_element_type = typename std::iterator_traits<ForwardIter>::value_type;
    using point_type         = std::invoke_result_t<PointMap, input_element_type>;
    using scalar_type        = typename point_type::coordinate_type;

    static_assert(
        traits::is_point_map_v<PointMap, input_element_type>,
        "point_map must satisfy PointMap concept");

    aabb_type aabb{};
    aabb.min.x(std::numeric_limits<scalar_type>::max());
    aabb.min.y(std::numeric_limits<scalar_type>::max());
    aabb.min.z(std::numeric_limits<scalar_type>::max());
    aabb.max.x(std::numeric_limits<scalar_type>::lowest());
    aabb.max.y(std::numeric_limits<scalar_type>::lowest());
    aabb.max.z(std::numeric_limits<scalar_type>::lowest());

    aabb = std::accumulate(begin, end, aabb, [&](aabb_type& bbox, input_element_type const& e) {
        point_type const& p = point_map(e);

        if (p.x() < bbox.min.x())
            bbox.min.x(p.x());
        if (p.y() < bbox.min.y())
            bbox.min.y(p.y());
        if (p.z() < bbox.min.z())
            bbox.min.z(p.z());
        if (p.x() > bbox.max.x())
            bbox.max.x(p.x());
        if (p.y() > bbox.max.y())
            bbox.max.y(p.y());
        if (p.z() > bbox.max.z())
            bbox.max.z(p.z());

        return bbox;
    });

    return aabb;
}

} // namespace pcp

#endif // PCP_COMMON_AXIS_ALIGNED_BOUNDING_BOX_HPP