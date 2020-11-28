#pragma once

#include "octree_node.hpp"
#include "point.hpp"
#include "traits/range_traits.hpp"

namespace pcp {

/**
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
 * @tparam PointView Type satisfying PointView concept
 * @tparam ParamsType Type containing the parameters for this octree
 */
template <class PointView, class ParamsType = octree_parameters_t<point_t>>
class basic_octree_t
{
  public:
    using octree_node_type = basic_octree_node_t<PointView, ParamsType>;
    using point_view_type  = PointView;
    using params_type      = ParamsType;
    using coordinate_type  = typename point_view_type::coordinate_type;
    using aabb_type        = typename ParamsType::aabb_type;
    using aabb_point_type  = typename aabb_type::point_type;
    using iterator         = octree_iterator_t<point_view_type, ParamsType>;
    using const_iterator   = iterator const;
    using value_type       = point_view_type;
    using reference        = value_type&;
    using const_reference  = value_type const&;
    using pointer          = value_type*;
    using const_pointer    = value_type const*;
    using self_type        = basic_octree_t<point_view_type, params_type>;

    basic_octree_t(self_type&& other) = default;

    explicit basic_octree_t(params_type const& params) : root_(params), size_(0u) {}

    template <class ForwardIter>
    explicit basic_octree_t(ForwardIter begin, ForwardIter end, params_type const& params)
        : root_(params), size_(root_.insert(begin, end))
    {
    }

    std::size_t size() const { return size_; }
    bool empty() const { return size() == 0u; }
    void clear() { root_.clear(); }
    iterator begin() { return iterator(&root_); }
    iterator end() { return iterator{}; }
    const_iterator cbegin() const { return const_iterator(const_cast<octree_node_type*>(&root_)); }
    const_iterator cend() const { return const_iterator{}; }

    template <class ForwardIter>
    std::size_t insert(ForwardIter begin, ForwardIter end)
    {
        static_assert(
            traits::is_point_view_v<std::remove_cv_t<typename ForwardIter::value_type>>,
            "ForwardIter::value_type must satisfy PointView concept");
        auto const inserted = root_.insert(begin, end);
        size_ += inserted;
        return inserted;
    }

    bool insert(point_view_type const& p)
    {
        bool const inserted = root_.insert(p);
        if (inserted)
            ++size_;

        return inserted;
    }

    /*
     * Returns an iterator to the point p in the octree if it exists.
     *
     * @param p Point to search for in the octree
     * @return iterator to the found point in the octree, or end iterator if it was not found
     */
    const_iterator find(point_view_type const& p) const { return root_.find(p); }

    /*
     * Removes the point pointed-to by iterator pos.
     *
     * @param pos Iterator to the point to remove. Must be in the range [cbegin(), cend()).
     * @return Iterator to the next point or cend()
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
     * @param k         The number of neighbors to return that are nearest to the specified point
     * for all points of the octree
     * @param target    The reference point for which we want the k nearest neighbors
     * @return A list of nearest points ordered from nearest to furthest of size s where 0 <= s <= k
     */
    std::vector<point_view_type> nearest_neighbours(
        point_view_type const& target,
        std::size_t k,
        coordinate_type eps = static_cast<coordinate_type>(1e-5)) const
    {
        return root_.nearest_neighbours(target, k, eps);
    }

    /*
     * Returns all points that reside in the given range.
     * The implementation is recursive.
     *
     * @param range A range satisfying the Range type requirements
     * @return A list of all points that reside in the given range
     */
    template <class Range>
    std::vector<point_view_type> range_search(Range const& range) const
    {
        static_assert(
            traits::is_range_v<Range, point_view_type>,
            "Range must satisfy Range concept");
        std::vector<point_view_type> points_in_range;
        root_.range_search(range, points_in_range);
        return points_in_range;
    }

  private:
    octree_node_type root_;
    std::size_t size_;
};

using octree_t = pcp::basic_octree_t<pcp::point_t>;

template <class OctreeIterator, class PointView>
OctreeIterator find(OctreeIterator first, OctreeIterator last, PointView const& value)
{
    static_assert(
        pcp::traits::is_point_view_v<PointView>,
        "PointView must satisfy PointView concept");
    static_assert(
        std::is_same_v<
            std::remove_cv_t<OctreeIterator>,
            std::remove_cv_t<typename pcp::basic_octree_t<PointView>::const_iterator>>,
        "InputIt must be octree iterator");
    if (first == last)
        return last;

    auto const* root = first.root();
    return root->find(value);
}

template <class OctreeIterator, class PointView>
auto count(OctreeIterator first, OctreeIterator last, PointView const& value) ->
    typename OctreeIterator::difference_type
{
    static_assert(
        pcp::traits::is_point_view_v<PointView>,
        "PointView must satisfy PointView concept");
    static_assert(
        std::is_same_v<
            std::remove_cv_t<OctreeIterator>,
            std::remove_cv_t<typename pcp::basic_octree_t<PointView>::const_iterator>>,
        "InputIt must be octree iterator");
    if (first == last)
        return 0u;

    auto const* root = first.root();
    std::vector<PointView> points;
    root->range_search(pcp::axis_aligned_bounding_box_t<pcp::point_t>{value, value}, points);
    return points.size();
}

} // namespace pcp
