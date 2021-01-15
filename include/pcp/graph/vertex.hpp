#ifndef PCP_GRAPH_VERTEX_HPP
#define PCP_GRAPH_VERTEX_HPP

/**
 * @file
 * @ingroup graph
 */

#include <cstdint>

namespace pcp {
namespace graph {

/**
 * @ingroup graph-structures-types
 * @brief
 * Most basic vertex type. It only stores an id.
 * @tparam Integer Type of the vertex's id
 */
template <class Integer = std::uint32_t>
class vertex_t
{
  public:
    using id_type   = Integer;
    using self_type = vertex_t<Integer>;

    vertex_t() = default;
    vertex_t(id_type id) : id_(id) {}

    void id(id_type id) { id_ = id; }
    id_type id() const { return id_; }

    bool operator==(self_type const& other) const { return id_ == other.id_; }
    bool operator==(id_type id) const { return id_ == id; }

  private:
    id_type id_ = 0u;
};

} // namespace graph
} // namespace pcp

#endif // PCP_GRAPH_VERTEX_HPP