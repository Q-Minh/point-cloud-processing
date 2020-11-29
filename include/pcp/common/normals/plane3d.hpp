#pragma once

#include <pcp/common/norm.hpp>
#include <pcp/common/vector3d.hpp>
#include <pcp/common/vector3d_queries.hpp>
#include <pcp/traits/normal_traits.hpp>
#include <pcp/traits/point_traits.hpp>

namespace pcp {
namespace common {

/**
 * @brief Simple 3d plane in euclidean space
 * @tparam Point Type of point used by the plane
 * @tparam Normal Type of normal used by the plane
 */
template <class Point, class Normal>
class plane3d_t
{
    static_assert(traits::is_point_v<Point>, "Point must satisfy Point concept");
    static_assert(traits::is_normal_v<Normal>, "Normal must satisfy Normal concept");

  public:
    using point_type     = Point;
    using normal_type    = Normal;
    using component_type = typename point_type::component_type;
    using self_type      = plane3d_t<Point, Normal>;

    plane3d_t()                 = default;
    plane3d_t(self_type const&) = default;
    plane3d_t(self_type&&)      = default;
    plane3d_t(point_type const& p, normal_type const& n) : point_(p), normal_(n) {}

    normal_type const& normal() const { return normal_; }
    point_type const& point() const { return point_; }

    void normal(normal_type const& n) { normal_ = n; }
    void point(point_type const& p) { point_ = p; }

    template <class PointView>
    component_type signed_distance_to(PointView const& p) const
    {
        vector3d_t<component_type> const d = p - point_;
        auto const ip                      = inner_product(d, normal_);
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
    normal_type normal_;
    point_type point_;
};

} // namespace common
} // namespace pcp