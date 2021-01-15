#ifndef PCP_COMMON_POINTS_POINT_VIEW_HPP
#define PCP_COMMON_POINTS_POINT_VIEW_HPP

/**
 * @file
 * @ingroup common
 */

#include "pcp/traits/point_traits.hpp"
#include "point.hpp"

namespace pcp {

/**
 * @ingroup geometric-primitives
 * @brief
 * Non-owning point used as a view over a given point.
 * The basic_point_view_t need only store a pointer to
 * the underlying point and exposes an interface used
 * to query the underlying point's coordinates.
 * @tparam PointView Type satisfying PointView concept
 */
template <class PointView>
class basic_point_view_t
{
    using point_type = PointView;
    static_assert(traits::is_point_view_v<point_type>, "PointView must satisfy PointView concept");

  public:
    using self_type       = basic_point_view_t<point_type>;
    using component_type  = typename point_type::component_type;
    using coordinate_type = typename point_type::coordinate_type;
    using T               = component_type;

    explicit basic_point_view_t(point_type* point) noexcept : point_(point) {}
    /**
     * @brief
     * Default constructing a basic_point_view_t is possible,
     * but beware of using it before having set its underlying
     * pointed-to point.
     */
    basic_point_view_t() noexcept                       = default;
    basic_point_view_t(self_type const& other) noexcept = default;
    basic_point_view_t(self_type&& other) noexcept      = default;
    self_type& operator=(self_type const& other) noexcept = default;
    self_type& operator=(self_type&& other) noexcept = default;

    template <class TPointView>
    self_type& operator=(TPointView const& other)
    {
        static_assert(
            traits::is_point_view_v<TPointView>,
            "TPointView must satisfy PointView concept");
        x(other.x());
        y(other.y());
        z(other.z());
    }

    T const& x() const { return point_->x(); }
    T const& y() const { return point_->y(); }
    T const& z() const { return point_->z(); }

    void x(T value) { point_->x(value); }
    void y(T value) { point_->y(value); }
    void z(T value) { point_->z(value); }

    void point(point_type* point) noexcept { point_ = point; }
    point_type const* point() const noexcept { return point_; }
    point_type* point() noexcept { return point_; }

  private:
    point_type* point_;
};

/**
 * @ingroup geometric-primitives
 * @brief
 * Default point view type
 */
using point_view_t = pcp::basic_point_view_t<pcp::point_t>;

} // namespace pcp

#endif // PCP_COMMON_POINTS_POINT_VIEW_HPP