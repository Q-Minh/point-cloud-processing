#ifndef PCP_COMMON_MESH_TRIANGLE_HPP
#define PCP_COMMON_MESH_TRIANGLE_HPP

#include <array>

namespace pcp {
namespace common {

/**
 * @brief
 * shared_vertex_mesh_triangle is encoded as a shared vertex mesh triangle.
 * shared_vertex_mesh_triangle need only store indices to vertices in a
 * mesh.
 * @tparam Integer Type of triangle indices to use.
 */
template <class Integer>
class shared_vertex_mesh_triangle
{
  public:
    using index_type   = Integer;
    using self_type    = shared_vertex_mesh_triangle<Integer>;
    using indices_type = std::array<index_type, 3>;

    shared_vertex_mesh_triangle() noexcept                 = default;
    shared_vertex_mesh_triangle(self_type const&) noexcept = default;
    shared_vertex_mesh_triangle(self_type&&) noexcept      = default;
    self_type& operator=(self_type const&) = default;
    self_type& operator=(self_type&&) = default;

    shared_vertex_mesh_triangle(index_type v1, index_type v2, index_type v3) : indices_{v1, v2, v3} {}

    indices_type const& indices() const { return indices_; }
    indices_type& indices() { return indices_; }

  private:
    indices_type indices_;
};

} // namespace common
} // namespace pcp

#endif // PCP_COMMON_MESH_TRIANGLE_HPP