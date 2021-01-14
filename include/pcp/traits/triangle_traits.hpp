#ifndef PCP_TRAITS_TRIANGLE_TRAITS_HPP
#define PCP_TRAITS_TRIANGLE_TRAITS_HPP

#include <type_traits>

namespace pcp {
namespace traits {

template <class SharedVertexMeshTriangle, class = void>
struct is_shared_vertex_mesh_triangle : std::false_type
{
};

template <class SharedVertexMeshTriangle>
struct is_shared_vertex_mesh_triangle<
    SharedVertexMeshTriangle,
    std::void_t<
        typename SharedVertexMeshTriangle::index_type,
        decltype(std::declval<SharedVertexMeshTriangle&>().indices())>> : std::true_type
{
    static_assert(
        std::is_constructible_v<
            SharedVertexMeshTriangle,
            typename SharedVertexMeshTriangle::index_type,
            typename SharedVertexMeshTriangle::index_type,
            typename SharedVertexMeshTriangle::index_type>,
        "SharedVertexMeshTriangle must be constructible from 3 indices as "
        "SharedVertexMeshTriangle(id1, id2, id3)");
};

template <class SharedVertexMeshTriangle>
static constexpr bool is_shared_vertex_mesh_triangle_v =
    is_shared_vertex_mesh_triangle<SharedVertexMeshTriangle>::value;

} // namespace traits
} // namespace pcp

#endif // PCP_TRAITS_TRIANGLE_TRAITS_HPP