#ifndef PCP_TRAITS_VECTOR3D_TRAITS_HPP
#define PCP_TRAITS_VECTOR3D_TRAITS_HPP

#include <type_traits>

namespace pcp {
namespace traits {

template <class Vector3d, class = void>
struct is_vector3d : std::false_type
{
};

template <class Vector3d>
struct is_vector3d<
    Vector3d,
    std::void_t<
        typename Vector3d::component_type,
        decltype(std::declval<Vector3d&>().x()),
        decltype(std::declval<Vector3d&>().y()),
        decltype(std::declval<Vector3d&>().z()),
        decltype(std::declval<typename Vector3d::component_type>() * std::declval<Vector3d&>()),
        decltype(std::declval<Vector3d&>() / std::declval<typename Vector3d::component_type>()),
        decltype(std::declval<Vector3d&>() + std::declval<Vector3d&>()),
        decltype(std::declval<Vector3d&>() - std::declval<Vector3d&>())>> : std::true_type
{
    static_assert(
        std::is_constructible_v<
            Vector3d,
            typename Vector3d::component_type,
            typename Vector3d::component_type,
            typename Vector3d::component_type>,
        "Vector3d must be constructible from x,y,z components");
};

template <class Vector3d>
static constexpr bool is_vector3d_v = is_vector3d<Vector3d>::value;

} // namespace traits
} // namespace pcp

#endif // PCP_TRAITS_VECTOR3D_TRAITS_HPP