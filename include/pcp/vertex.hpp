#pragma once

#include "point.hpp"
#include "point_view.hpp"
#include "traits/graph_vertex_traits.hpp"
#include "traits/point_traits.hpp"

#include <cstdint>

namespace pcp {

/**
 * @brief
 * The basic_vertex_t type is a basic_point_view_t. It does not
 * hold ownership over the underlying point. It stores an
 * identifier, and exposes it through id() and id(id_type).
 * basic_vertex_t should satify the GraphVertex concept.
 * Use this type as an adaptor to points so that they are
 * useable in graph algorithms without copying.
 * @tparam PointView Type satisfying PointView concept
 */
template <class PointView>
class basic_vertex_t : public basic_point_view_t<PointView>
{
  public:
    using id_type     = std::uint64_t;
    using self_type   = basic_vertex_t<PointView>;
    using point_type  = basic_point_view_t<PointView>;
    using parent_type = point_type;

    id_type id() const { return id_; }
    void id(id_type value) { id_ = value; }

    basic_vertex_t() = default;

    explicit basic_vertex_t(PointView* point) : parent_type(point), id_(0u) {}
    explicit basic_vertex_t(id_type id) : parent_type(), id_(id) {}
    explicit basic_vertex_t(PointView* point, id_type id) : parent_type(point), id_(id) {}
    basic_vertex_t(point_type const& other) : parent_type(other), id_(0u) {}

    /**
     * @brief
     * Construct from another GraphVertex type. This
     * constructor copies only the id from the copied-from
     * object.
     * @tparam GraphVertex Type satisfying GraphVertex
     * @param other GraphVertex to copy from
     */
    template <class GraphVertex>
    basic_vertex_t(GraphVertex const& other) : parent_type(), id_(other.id())
    {
        static_assert(
            traits::is_graph_vertex_v<GraphVertex>,
            "other must satisfy GraphVertex concept");
    }

    /**
     * @brief
     * Assigns id of other GraphVertex to this basic_vertex_t.
     * If other is also a PointView, also assign x,y,z coordinates.
     * @tparam GraphVertex Type satisfying GraphVertex concept
     * @param other GraphVertex to assign from
     * @return
     */
    template <class GraphVertex>
    self_type& operator=(GraphVertex const& other)
    {
        static_assert(
            traits::is_graph_vertex_v<GraphVertex>,
            "GraphVertex must satisfy GraphVertex concept");
        id(other.id());

        if constexpr (traits::is_point_view_v<GraphVertex>)
        {
            x(other.x());
            y(other.y());
            z(other.z());
        }

        return *this;
    }

    /**
     * @brief Equality comparison based on identifier
     * @param other
     * @return
     */
    bool operator==(self_type const& other) const { return id_ == other.id_; }
    /**
     * @brief Inequality comparison based on identifier
     * @param other
     * @return
     */
    bool operator!=(self_type const& other) const { return id_ != other.id_; }

  private:
    id_type id_ = 0u;
};

using vertex_t = basic_vertex_t<pcp::point_t>;

} // namespace pcp