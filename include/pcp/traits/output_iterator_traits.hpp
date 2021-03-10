#ifndef PCP_TRAITS_OUTPUT_ITERATOR_TRAITS_HPP
#define PCP_TRAITS_OUTPUT_ITERATOR_TRAITS_HPP

/**
 * @file
 * @ingroup traits
 */

/**
 * Taken from:
 * https://stackoverflow.com/questions/29065760/traits-class-to-extract-containers-value-type-from-a-back-insert-iterator
 */

// clang-format off
#include <iterator>             // iterator, iterator_traits, input_iterator_tag, output_iterator_tag, random_access_iterator_tag
                                // back_insert_iterator, front_insert_iterator, insert_iterator, ostream_iterator, ostreambuf_iterator
// clang-format on

namespace xstd {

template <class T>
struct output_iterator_traits : std::iterator_traits<T>
{
};

template <class Container>
struct output_iterator_traits<std::back_insert_iterator<Container>>
{
    using value_type      = typename Container::value_type;
    using difference_type = typename Container::difference_type;
    using pointer         = typename Container::difference_type;
    using reference       = typename Container::reference;
    using iterator_tag    = std::output_iterator_tag;
};

template <class Container>
struct output_iterator_traits<std::front_insert_iterator<Container>>
{
    using value_type      = typename Container::value_type;
    using difference_type = typename Container::difference_type;
    using pointer         = typename Container::difference_type;
    using reference       = typename Container::reference;
    using iterator_tag    = std::output_iterator_tag;
};

template <class Container>
struct output_iterator_traits<std::insert_iterator<Container>>
{
    using value_type      = typename Container::value_type;
    using difference_type = typename Container::difference_type;
    using pointer         = typename Container::difference_type;
    using reference       = typename Container::reference;
    using iterator_tag    = std::output_iterator_tag;
};

} // namespace xstd

#endif // PCP_TRAITS_OUTPUT_ITERATOR_TRAITS_HPP