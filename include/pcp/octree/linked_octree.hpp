#ifndef PCP_OCTREE_LINKED_OCTREE_HPP
#define PCP_OCTREE_LINKED_OCTREE_HPP

/**
 * @file
 * @ingroup octree
 */

#include "linked_octree_node.hpp"
#include "pcp/algorithm/common.hpp"
#include "pcp/common/points/point.hpp"
#include "pcp/traits/property_map_traits.hpp"
#include "pcp/traits/range_traits.hpp"

#include <range/v3/view/subrange.hpp>
#include <range/v3/view/transform.hpp>

namespace pcp {

/**
 * @ingroup linked-octree
 * @brief
 * An octree is a tree data structure for 3-dimensional quantities which
 * recursively subdivides a regular grid into its 8 octants by having
 * 8 child octrees. Octrees, much like binary trees, offer efficient
 * searching and insertion of those 3-d quantities (logarithmic time).
 * Interesting applications of octrees include efficient collision detection,
 * k-nearest-neighbor searches and range queries, among others.
 *
 * Our octree implementation is sparse, as it only grows nodes when
 * the number of vertices in a parent node exceeds its configured
 * capacity to reduce memory usage. Vertices are not all stored at
 * the same level. The implementation is pointer-based to create
 * a linked tree structure. The octree is also dynamic, so erasing
 * points from the octree is possible.
 *
 * @tparam Element Type of the octree's elements
 * @tparam ParamsType Type containing the parameters for this octree
 */
template <class Element, class ParamsType = octree_parameters_t<pcp::point_t>>
class basic_linked_octree_t
{
  public:
    using element_type = Element; ///< Type of the elements stored by this octree
    using octree_node_type =
        basic_linked_octree_node_t<Element, ParamsType>; ///< Type of node stored
    using params_type = ParamsType;                      ///< Type of the octree's parameters
    using aabb_type   = typename ParamsType::aabb_type;  ///< Type of AABB used to represent voxels
    using aabb_point_type = typename aabb_type::point_type; ///< Type of point used by the AABB
    using iterator =
        linked_octree_iterator_t<Element, ParamsType>; ///< iterator type of this octree
    using const_iterator  = iterator const;
    using value_type      = element_type;
    using reference       = value_type&;
    using const_reference = value_type const&;
    using pointer         = value_type*;
    using const_pointer   = value_type const*;
    using self_type = basic_linked_octree_t<element_type, params_type>; ///< Type of this octree

    /**
     * @brief Default move constructor
     * @param other Copied-from octree
     */
    basic_linked_octree_t(self_type&& other) = default;

    /**
     * @brief
     * Constructs this octree with configuration specified by params
     * @param params The configuration for this octree
     */
    explicit basic_linked_octree_t(params_type const& params) : root_(params), size_(0u) {}

    /**
     * @brief
     * Constructs this octree from a range of elements
     * @tparam ForwardIter Type of iterator to the elements
     * @tparam PointViewMap Type satisfying PointViewMap concept
     * @param begin Begin iterator to the elements
     * @param end End iterator to the elements
     * @param point_view The point view property map
     * @param params The configuration for this octree
     */
    template <class ForwardIter, class PointViewMap>
    explicit basic_linked_octree_t(
        ForwardIter begin,
        ForwardIter end,
        PointViewMap const& point_view,
        params_type const& params)
        : root_(params), size_(root_.insert(begin, end, point_view))
    {
    }

    /**
     * @brief
     * Constructs this octree from a range of elements.
     * Computes the octree's configuration automatically.
     * @tparam ForwardIter Type of iterator to the elements
     * @tparam PointViewMap Type satisfying PointViewMap concept
     * @param begin Begin iterator to the elements
     * @param end The point view property map
     * @param point_view The point view property map
     */
    template <class ForwardIter, class PointViewMap>
    explicit basic_linked_octree_t(
        ForwardIter begin,
        ForwardIter end,
        PointViewMap const& point_view)
        : root_{}, size_{}
    {
        params_type params;
        auto const projection = [&](element_type const& e) {
            return point_view(e);
        };
        auto rng = ranges::make_subrange(begin, end) | ranges::views::transform(projection);
        using rng_iter_type = decltype(rng.begin());
        auto const bbox =
            pcp::bounding_box<rng_iter_type, aabb_point_type, aabb_type>(rng.begin(), rng.end());
        params.voxel_grid = bbox;
        root_             = octree_node_type{params};
        size_             = root_.insert(begin, end, point_view);
    }

    /**
     * @brief Number of elements in the octree
     * @return Number of elements in the octree
     */
    std::size_t size() const { return size_; }

    /**
     * @brief Checks if octree is empty
     * @return True if octree is empty
     */
    bool empty() const { return size() == 0u; }

    /**
     * @brief Remove all elements from this octree
     */
    void clear() { root_.clear(); }

    /**
     * @brief Gets the top-level voxel from this octree (the bounding box)
     * @return This octree's root voxel
     */
    aabb_type const& voxel_grid() const { return root_.voxel_grid(); }

    /**
     * @brief Iterator to the first element of this octree
     * @return Iterator to the first element of this octree
     */
    iterator begin() { return iterator(&root_); }

    /**
     * @brief End iterator to this octree's elements
     * @return End iterator to this octree's elements
     */
    iterator end() { return iterator{}; }

    /**
     * @brief Const iterator to the first element of this octree
     * @return Const iterator to the first element of this octree
     */
    const_iterator cbegin() const { return const_iterator(const_cast<octree_node_type*>(&root_)); }

    /**
     * @brief End const iterator to this octree's elements
     * @return End const iterator to this octree's elements
     */
    const_iterator cend() const { return const_iterator{}; }

    /**
     * @brief Insert range of elements in the octree
     * @tparam ForwardIter Type of the range's iterators
     * @tparam PointViewMap Type satisfying PointViewMap concept
     * @param begin Iterator to the first element of the range
     * @param end End iterator of the range
     * @param point_view The point view property map
     * @return The number of inserted elements
     */
    template <class ForwardIter, class PointViewMap>
    std::size_t insert(ForwardIter begin, ForwardIter end, PointViewMap const& point_view)
    {
        static_assert(
            traits::is_point_view_v<std::remove_cv_t<typename ForwardIter::value_type>>,
            "ForwardIter::value_type must satisfy PointView concept");
        auto const inserted =
            root_.template insert<ForwardIter, PointViewMap>(begin, end, point_view);
        size_ += inserted;
        return inserted;
    }

    /**
     * @brief Insert one element in the octree
     * @tparam PointViewMap Type satisfying PointViewMap concept
     * @param e The element to insert
     * @param point_view The point view property map
     * @return true if insert was successful
     */
    template <class PointViewMap>
    bool insert(element_type const& e, PointViewMap const& point_view)
    {
        bool const inserted = root_.template insert<PointViewMap>(e, point_view);
        if (inserted)
            ++size_;

        return inserted;
    }

    /*
     * Returns an iterator to the point p in the octree if it exists.
     *
     * @param e Element to search for in the octree
     * @param point_view The PointViewMap property map
     * @return iterator to the found element in the octree, or end iterator if it was not found
     */
    template <class PointViewMap>
    const_iterator find(element_type const& e, PointViewMap const& point_view) const
    {
        return root_.template find<PointViewMap>(e, point_view);
    }

    /*
     * Removes the element pointed-to by iterator pos.
     *
     * @param pos Iterator to the element to remove. Must be in the range [cbegin(), cend()).
     * @return Iterator to the next element or cend()
     */
    const_iterator erase(const_iterator pos)
    {
        --size_;
        return root_.erase(pos);
    }

    /*
     * Returns the k-nearest-neighbours in 3d Euclidean space
     * using the l2-norm as the notion of distance.
     * The implementation is recursive.
     *
     * @param target    The reference point for which we want the k nearest neighbors
     * @param k         The number of neighbors to return that are nearest to the specified point
     * for all points of the octree
     * @param point_view The PointViewMap property map
     * @param eps The error tolerance for floating point equality
     * @return A list of nearest points ordered from nearest to furthest of size s where 0 <= s <= k
     */
    template <class TPointView, class PointViewMap>
    std::vector<element_type> nearest_neighbours(
        TPointView const& target,
        std::size_t k,
        PointViewMap const& point_view,
        double eps = 1e-5) const
    {
        return root_
            .template nearest_neighbours<TPointView, PointViewMap>(target, k, point_view, eps);
    }

    /*
     * Returns all points that reside in the given range.
     * The implementation is recursive.
     *
     * @param range A range satisfying the Range type requirements
     * @param point_view The PointViewMap property map
     * @return A list of all points that reside in the given range
     */
    template <class Range, class PointViewMap>
    std::vector<element_type> range_search(Range const& range, PointViewMap const& point_view) const
    {
        using point_view_type =
            typename traits::property_map_traits<PointViewMap, Element>::value_type;

        static_assert(
            traits::is_range_v<Range, point_view_type>,
            "Range must satisfy Range concept");
        std::vector<element_type> elements_in_range;
        root_.template range_search<Range, PointViewMap>(range, elements_in_range, point_view);
        return elements_in_range;
    }

  private:
    octree_node_type root_;
    std::size_t size_;
};

using linked_octree_t = pcp::basic_linked_octree_t<pcp::point_t>;

} // namespace pcp

#endif // PCP_OCTREE_LINKED_OCTREE_HPP