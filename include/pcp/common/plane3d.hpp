#ifndef PCP_COMMON_PLANE3D_HPP
#define PCP_COMMON_PLANE3D_HPP

/**
 * @file
 * @ingroup common
 */

#include "pcp/common/norm.hpp"
#include "pcp/common/normals/normal.hpp"
#include "pcp/common/normals/normal_estimation.hpp"
#include "pcp/common/points/point.hpp"
#include "pcp/common/vector3d.hpp"
#include "pcp/common/vector3d_queries.hpp"
#include "pcp/traits/normal_traits.hpp"
#include "pcp/traits/plane_traits.hpp"
#include "pcp/traits/point_traits.hpp"

namespace pcp {
namespace common {

/**
 * @ingroup geometric-primitives
 * @brief Simple 3d plane in euclidean space
 * @tparam Point Type of point used by the plane
 * @tparam Normal Type of normal used by the plane
 */
template <class Point, class Normal>
class basic_plane3d_t
{
    static_assert(traits::is_point_v<Point>, "Point must satisfy Point concept");
    static_assert(traits::is_normal_v<Normal>, "Normal must satisfy Normal concept");

  public:
    using point_type     = Point;
    using normal_type    = Normal;
    using component_type = typename point_type::component_type;
    using self_type      = basic_plane3d_t<Point, Normal>;

    basic_plane3d_t() noexcept                 = default;
    basic_plane3d_t(self_type const&) noexcept = default;
    basic_plane3d_t(self_type&&) noexcept      = default;
    self_type& operator=(self_type const&) noexcept = default;
    self_type& operator=(self_type&&) noexcept = default;
    basic_plane3d_t(point_type const& p, normal_type const& n) : point_(p), normal_(n) {}

    normal_type const& normal() const { return normal_; }
    point_type const& point() const { return point_; }

    void normal(normal_type const& n) { normal_ = n; }
    void point(point_type const& p) { point_ = p; }

    template <class PointView>
    component_type signed_distance_to(PointView const& p) const
    {
        basic_vector3d_t<component_type> const d = p - point_;
        auto const ip                            = inner_product(d, normal_);
        return ip;
    }

    template <class PointView>
    bool contains(PointView const& p, component_type eps = static_cast<component_type>(1e-5)) const
    {
        auto const d        = signed_distance_to(p);
        using distance_type = decltype(d);
        auto const zero     = static_cast<distance_type>(0.0);
        return floating_point_equals(d, zero, eps);
    }

  private:
    point_type point_;
    normal_type normal_;
};

/**
 * @ingroup geometric-primitives
 * @brief
 * Default plane type
 */
using plane3d_t = pcp::common::basic_plane3d_t<pcp::point_t, pcp::normal_t>;

/**
 * @ingroup common
 * @brief Computes an estimated tangent plane from a sequence of 3d points
 * @tparam ForwardIter Iterator to points
 * @tparam PointMap Type satisfying PointMap concept
 * @tparam Plane Type of plan to compute
 * @param begin Iterator to start of the sequence of elements
 * @param end Iterator to one past the end of the sequence of elements
 * @param point_map The point map property map
 * @return
 */
template <class ForwardIter, class PointMap, class Plane = pcp::common::plane3d_t>
Plane tangent_plane(ForwardIter begin, ForwardIter end, PointMap const& point_map)
{
    static_assert(traits::is_plane_v<Plane>, "Plane must satisfy Plane concept");

    using normal_type = typename Plane::normal_type;
    using point_type  = typename Plane::point_type;

    normal_type normal = estimate_normal<ForwardIter, PointMap, normal_type>(begin, end, point_map);
    point_type point   = center_of_geometry<ForwardIter, PointMap>(begin, end, point_map);

    return Plane(point, normal);
}

} // namespace common
} // namespace pcp

#endif // PCP_COMMON_PLANE3D_HPP
