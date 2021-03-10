#ifndef PCP_ALGORITHM_RANDOM_SIMPLIFICATION_HPP
#define PCP_ALGORITHM_RANDOM_SIMPLIFICATION_HPP

/**
 * @file
 * @ingroup algorithm
 */

#include <algorithm>
#include <numeric>
#include <random>

namespace pcp {
namespace algorithm {

/**
 * @ingroup point-cloud-simplification
 * @brief Randomly conserves only a subset of the input point set
 * @tparam RandomAccessIter Iterator type of input point set
 * @tparam OutputIter Iterator type of output point set
 * @param begin Start iterator of input point set
 * @param end End iterator of input point set
 * @param out_begin Start iterator of output point set
 * @param output_size Requested size of output point set
 * @param use_indices Shuffle a vector of indices to the input point set if true. This uses
 * additional memory to handle the indices. Otherwise, shuffle the input point set directly
 * @return End iterator to output point set
 */
template <class RandomAccessIter, class OutputIter>
OutputIter random_simplification(
    RandomAccessIter begin,
    RandomAccessIter end,
    OutputIter out_begin,
    std::size_t const output_size,
    bool const use_indices = true)
{
    using difference_type = typename std::iterator_traits<RandomAccessIter>::difference_type;
    std::size_t const N   = static_cast<std::size_t>(std::distance(begin, end));
    assert(output_size <= N);

    std::random_device rd{};
    std::mt19937 generator{rd()};

    if (use_indices)
    {
        std::vector<std::size_t> indices(N);
        std::iota(indices.begin(), indices.end(), 0u);
        std::shuffle(indices.begin(), indices.end(), generator);
        return std::transform(
            indices.begin(),
            indices.begin() + static_cast<difference_type>(output_size),
            out_begin,
            [&](std::size_t const i) { return *(begin + static_cast<difference_type>(i)); });
    }

    std::shuffle(begin, end, generator);
    return std::copy(begin, begin + static_cast<difference_type>(output_size), out_begin);
}

} // namespace algorithm
} // namespace pcp

#endif // PCP_ALGORITHM_RANDOM_SIMPLIFICATION_HPP
