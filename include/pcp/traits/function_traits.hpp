#ifndef PCP_TRAITS_FUNCTION_TRAITS_HPP
#define PCP_TRAITS_FUNCTION_TRAITS_HPP

/**
 * @file
 * @ingroup traits
 */

#include <type_traits>

namespace pcp {
namespace traits {

template <class Func, class Scalar, class = void>
struct is_3d_scalar_function : std::false_type
{
};

/**
 * @ingroup traits
 * @brief
 * ScalarFunc3D concept
 * @tparam Func Callable type taking 3 Scalar types as parameters
 * @tparam Scalar Arithmetic type
 */
template <class Func, class Scalar>
struct is_3d_scalar_function<
    Func,
    Scalar,
    std::void_t<decltype(
        std::declval<
            Func&>()(std::declval<Scalar&>(), std::declval<Scalar&>(), std::declval<Scalar&>()))>>
    : std::true_type
{
    static_assert(std::is_arithmetic_v<Scalar>, "Scalar must be an arithmetic type");
};

/**
 * @ingroup traits
 * @brief
 * Compile-time check for ScalarFunc3D concept
 * @tparam Func
 * @tparam Scalar
 */
template <class Func, class Scalar = float>
static constexpr bool is_3d_scalar_function_v = is_3d_scalar_function<Func, Scalar>::value;

} // namespace traits
} // namespace pcp

#endif // PCP_TRAITS_FUNCTION_TRAITS_HPP