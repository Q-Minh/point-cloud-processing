#ifndef PCP_KDTREE_LINKED_KDTREE_HPP
#define PCP_KDTREE_LINKED_KDTREE_HPP

/**
 * @file
 * @ingroup kd-tree
 */

#include "pcp/common/axis_aligned_bounding_box.hpp"
#include "pcp/common/intersections.hpp"
#include "pcp/common/points/point.hpp"
#include "pcp/common/vector3d_queries.hpp"
#include "pcp/kdtree/linked_kdtree_node.hpp"
#include "pcp/traits/coordinate_map.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <stack>

namespace pcp {

namespace kdtree {
enum class construction_t { nth_element, presort };

struct construction_params_t
{
    std::size_t max_depth                           = 12u;
    construction_t construction                     = construction_t::nth_element;
    std::size_t min_element_count_for_parallel_exec = 32'768;
    bool compute_max_depth                          = false;
    std::size_t max_elements_per_leaf               = 64u;
};

} // namespace kdtree

/**
 * @ingroup linked-kd-tree
 * @brief
 * A kdtree is a tree data structure for K-dimensional quantities that is
 * very similar to a binary search tree but for k dimensions instead of 1 dimension.
 * The left child of the current node is always smaller than the current node on 1 dimension and
 * the right child of the current node is always greater than the current node on 1 dimension.
 * To compare, we simply alternate the dimension every
 * time we increase the depth. 
 * 
 * ex (k=2):
 * 
 * depth 0 => compare on the 1st dimension
 * 
 * depth 1 => compare on the 2nd dimension
 * 
 * depth 2 => compare on the 1st dimension again
 *
 * Our Kdtree implementation uses a flat storage of the elements
 * and the nodes contain pointers to the storage. The depth of the tree
 * is adjustable only on construction and in our implementation,
 * the leaf nodes contain one or more than elements.
 *
 * @tparam Element Type of the kdtree's elements
 * @tparam K Dimensionality of the stored elements
 * @tparam CoordinateMap The mapping between the index of an element and its coordinates
 */
template <class Element, std::size_t K, class CoordinateMap>
class basic_linked_kdtree_t
{
  public:
    using self_type        = basic_linked_kdtree_t;
    using element_type     = Element;
    using node_type        = basic_linked_kdtree_node_t<element_type>;
    using node_type_ptr    = typename node_type::self_type_ptr;
    using coordinates_type = std::invoke_result_t<CoordinateMap, Element>;
    using coordinate_type  = traits::coordinate_type<CoordinateMap, Element>;
    using aabb_type        = kd_axis_aligned_bounding_box_t<coordinate_type, K>;

    // container aliases
    using iterator        = typename std::vector<element_type>::iterator;
    using const_iterator  = typename std::vector<element_type>::const_iterator;
    using value_type      = element_type;
    using reference       = value_type&;
    using const_reference = value_type const&;
    using difference_type = typename std::vector<element_type>::difference_type;
    using size_type       = typename std::vector<element_type>::size_type;
    using allocator_type  = typename std::vector<element_type>::allocator_type;

    static_assert(
        traits::is_coordinate_map_v<CoordinateMap, Element, coordinate_type, K>,
        "CoordinateMap must satisfy CoordinateMap concept");

    /**
     * @brief
     * Constructs this kdtree from a range of elements and a coordinate map
     * @tparam ForwardIter
     * @param begin Begin iterator to the elements
     * @param end The point view property map
     * @param coordinate_map The coordinate map for the mapping between the element and its
     * coordinates
     * @param params The configuration for this kdtree
     */
    template <class ForwardIter>
    basic_linked_kdtree_t(
        ForwardIter begin,
        ForwardIter end,
        CoordinateMap coordinate_map         = CoordinateMap{},
        kdtree::construction_params_t params = kdtree::construction_params_t{})
        : max_depth_{params.max_depth},
          storage_(begin, end),
          root_{},
          coordinate_map_{coordinate_map},
          aabb_{}
    {
        aabb_ = kd_bounding_box<coordinate_type, K, CoordinateMap, ForwardIter>(
            begin,
            end,
            coordinate_map);

        if (params.compute_max_depth)
        {
            std::size_t const num_elements = storage_.size();
            auto const adaptive_max_depth  = std::log2(
                static_cast<double>(num_elements) /
                static_cast<double>(params.max_elements_per_leaf));
            max_depth_ = static_cast<std::size_t>(adaptive_max_depth);
        }

        if (params.construction == kdtree::construction_t::nth_element)
        {
            construct_nth_element(params.min_element_count_for_parallel_exec);
        }
        if (params.construction == kdtree::construction_t::presort)
        {
            // TODO: Perform correct presorting in exact median construction method
            // construct_presort();
        }
    }

    /**
     * @brief Checks if kdtree is empty
     * @return True if kdtree is empty
     *
     */
    bool empty() const { return storage_.empty(); }

    /**
     * @brief Number of elements in the kdtree
     * @return Number of elements in the kdtree
     */
    std::size_t size() const { return storage_.size(); }

    /**
     * @brief Removes all elements from the kdtree
     */
    void clear()
    {
        root_.reset();
        storage_.clear();
    }

    /**
     * @brief Iterator to the first element of this kdtree
     * @return Iterator to the first element of this kdtree
     */
    iterator begin() { return storage_.begin(); }

    /**
     * @brief End iterator to this kdtree's elements
     * @return  End iterator to this kdtree's elements
     */
    iterator end() { return storage_.end(); }

    /**
     * @brief Const iterator to the first element of this ktree
     * @return Const iterator to the first element of this kdtree
     */
    const_iterator cbegin() const { return storage_.cbegin(); }
    const_iterator cend() const { return storage_.cend(); }

    /**
     * @brief Root of the kdtree
     * @return Root of the kdtree
     */
    node_type_ptr const& root() const { return root_; }
    /**
     * @brief Bounding box of the kdtree
     * @return Bounding box of the kdtree
     */
    aabb_type const& aabb() const { return aabb_; }

    /**
     * @brief 
     * Returns the k-nearest-neighbours in K dimensions Euclidean space.
     * This algorithm will not return a point that is the same as the target point.
     * The implementation is recursive.
     * @param target the coordinates to the reference point for which we want the k nearest
     * neighbors
     * @param k The number of neighbors to return that are nearest to the specified point
     * @param eps eps The error tolerance for floating point equality
     * @return A list of nearest points ordered from nearest to furthest of size s where 0 <= s <= k
     */
    std::vector<element_type> nearest_neighbours(
        coordinates_type const& target,
        std::size_t k,
        coordinate_type eps = static_cast<coordinate_type>(1e-5)) const
    {
        auto const less_than_coordinates =
            [target](coordinates_type const& c1, coordinates_type const& c2) {
                auto const distance1 = common::squared_distance(target, c1);
                auto const distance2 = common::squared_distance(target, c2);
                return distance1 < distance2;
            };

        auto const less_than_elements = [less_than_coordinates,
                                         this](element_type const* e1, element_type const* e2) {
            auto const& coordinates1 = coordinate_map_(*e1);
            auto const& coordinates2 = coordinate_map_(*e2);

            return less_than_coordinates(coordinates1, coordinates2);
        };

        node_type const* current_node = root_.get();
        std::priority_queue<element_type*, std::vector<element_type*>, decltype(less_than_elements)>
            max_heap(less_than_elements);

        recurse_knn<decltype(less_than_coordinates), decltype(less_than_elements)>(
            target,
            k,
            current_node,
            aabb_,
            0u,
            less_than_coordinates,
            less_than_elements,
            max_heap,
            eps);

        std::vector<element_type> knearest_neighbours{};
        knearest_neighbours.reserve(k);
        while (!max_heap.empty())
        {
            knearest_neighbours.push_back(*max_heap.top());
            max_heap.pop();
        }
        std::reverse(knearest_neighbours.begin(), knearest_neighbours.end());
        return knearest_neighbours;
    }

    /**
     * @brief 
     * Returns the k-nearest-neighbours in K dimensions Euclidean space.
     * This algorithm will not return a point that is the same as the target point.
     * The implementation is recursive.
     * @param element_target The reference point for which we want the k nearest neighbors
     * @param k The number of neighbors to return that are nearest to the specified point
     * @param eps eps The error tolerance for floating point equality
     * @return A list of nearest points ordered from nearest to furthest of size s where 0 <= s <= k
     */
    std::vector<element_type> nearest_neighbours(
        element_type const& element_target,
        std::size_t k,
        coordinate_type eps = static_cast<coordinate_type>(1e-5)) const
    {
        coordinates_type const& target = coordinate_map_(element_target);
        return nearest_neighbours(target, k, eps);
    }

    /**
     * @brief Range search
     * @param range The range in which we want to find points
     * @return the points in the range
     */
    template <class Range>
    std::vector<element_type> range_search(Range const& range)
    {
        std::vector<element_type> elements_in_range{};
        node_type const* current_node = root_.get();
        range_search_recursive(range, aabb_, current_node, elements_in_range, 0);
        return elements_in_range;
    }

  private:
    template <class Range>
    void range_search_recursive(
        Range const& range,
        aabb_type const& current_aabb,
        node_type const* current_node,
        std::vector<element_type>& elements_in_range,
        std::size_t current_depth)
    {
        // verify if the point is in the range
        auto const node_elements = current_node->points();
        for (auto const& element : node_elements)
        {
            coordinates_type const& element_coordinates = coordinate_map_(*element);
            if (range.contains(element_coordinates))
                elements_in_range.push_back(*element);
        }
        auto const dimension      = current_depth % K;
        auto const& median        = current_node->points().front();
        auto const& median_point  = coordinate_map_(*median);
        aabb_type left_aabb       = current_aabb;
        left_aabb.max[dimension]  = median_point[dimension];
        aabb_type right_aabb      = current_aabb;
        right_aabb.min[dimension] = median_point[dimension];
        auto left_child           = current_node->left().get();
        auto right_child          = current_node->right().get();

        ++current_depth;
        if (left_child != nullptr && intersections::intersects(left_aabb, range))
            range_search_recursive(range, left_aabb, left_child, elements_in_range, current_depth);
        if (right_child != nullptr && intersections::intersects(right_aabb, range))
            range_search_recursive(
                range,
                right_aabb,
                right_child,
                elements_in_range,
                current_depth);
    }
    /**
     * @brief comparator for elements on a certain dimension
     * @tparam Element
     * @tparam CoordinateMap
     */
  public:
    struct less_than_t
    {
      public:
        less_than_t(std::size_t dimension, CoordinateMap const& coordinate_map)
            : dimension_{dimension}, coordinate_map_{coordinate_map} {};

        bool operator()(element_type const& e1, element_type const& e2) const
        {
            coordinates_type const& coordinates1 = coordinate_map_(e1);
            coordinates_type const& coordinates2 = coordinate_map_(e2);

            return coordinates1[dimension_] < coordinates2[dimension_];
        }

      private:
        std::size_t dimension_;
        CoordinateMap coordinate_map_;
    };

  private:
    void construct_nth_element(std::size_t min_element_count_for_parallel_exec)
    {
        auto size = storage_.size();
        root_ =
            construct_nth_element_recursive(min_element_count_for_parallel_exec, 0u, size - 1u, 0u);
    }

    std::unique_ptr<node_type> construct_nth_element_recursive(
        std::size_t min_element_count_for_parallel_exec,
        std::size_t first,
        std::size_t last,
        std::size_t current_depth)
    {
        /**
         * No left sub-tree for parent node
         */
        if (last < first)
        {
            return nullptr;
        }

        auto node    = std::make_unique<node_type>();
        auto& points = node->points();
        auto size    = std::size_t{(last + 1u) - first};

        /**
         * Leaf node
         */
        if (current_depth == max_depth_ - 1u)
        {
            points.resize(size);
            auto begin     = storage_.begin() + static_cast<difference_type>(first);
            auto end       = storage_.begin() + static_cast<difference_type>(last + 1u);
            auto out_begin = points.begin();

            std::transform(begin, end, out_begin, [](element_type& e) {
                return std::addressof(e);
            });
            return node;
        }

        /**
         * Leaf node
         */
        if (first == last)
        {
            points.push_back(std::addressof(storage_[first]));
            return node;
        }

        auto const dimension = current_depth % K;
        less_than_t const less_than{dimension, coordinate_map_};

        auto begin = storage_.begin() + static_cast<difference_type>(first);
        auto end   = storage_.begin() + static_cast<difference_type>(last + 1u);

        bool const parallelize = size >= min_element_count_for_parallel_exec;
        auto mid               = begin + static_cast<difference_type>(size / 2u);
        if (parallelize)
            std::nth_element(std::execution::par, begin, mid, end, less_than);
        else
            std::nth_element(std::execution::seq, begin, mid, end, less_than);

        auto median = first + size / 2u;
        points.push_back(std::addressof(storage_[median]));

        ++current_depth;
        auto left_child = construct_nth_element_recursive(
            min_element_count_for_parallel_exec,
            first,
            median - 1u,
            current_depth);
        auto right_child = construct_nth_element_recursive(
            min_element_count_for_parallel_exec,
            median + 1u,
            last,
            current_depth);
        node->set_left(std::move(left_child));
        node->set_right(std::move(right_child));

        return node;
    }

    /**
     * @brief Do not use.
     */
    void construct_presort() {}

    // std::unique_ptr<node_type>
    // construct_presort_recursive(std::size_t first, std::size_t last, std::size_t current_depth)
    //{
    //}

    template <class CoordinatesLessThanType, class ElementLessThanType>
    void recurse_knn(
        coordinates_type const& target,
        std::size_t k,
        node_type const* current_node,
        aabb_type const& current_aabb,
        std::size_t current_depth,
        CoordinatesLessThanType const& coordinates_less_than,
        ElementLessThanType const& element_less_than,
        std::priority_queue<element_type*, std::vector<element_type*>, ElementLessThanType>&
            max_heap,
        coordinate_type eps = static_cast<coordinate_type>(1e-5)) const
    {
        /**
         * Add elements of the current node in our current
         * best k nearest neighbours if those elements are
         * better k nearest neighbours (close than the max
         * heap's root)
         */
        auto const& elements = current_node->points();
        for (auto const& element : elements)
        {
            bool const is_heap_full = max_heap.size() == k;
            // TODO: Make array element checking code generated at compile time
            //       using possibly integer sequence, or other TMP techniques
            auto const are_kd_vectors_equal = [=](auto const& v1, auto const& v2) {
                auto rng = ranges::views::zip(v1, v2);
                return std::all_of(rng.begin(), rng.end(), [=](auto&& tup) {
                    auto const& c1 = std::get<0>(tup);
                    auto const& c2 = std::get<1>(tup);
                    return common::floating_point_equals(c1, c2, eps);
                });
            };
            auto const is_target = [&, this](element_type const& e) {
                coordinates_type const& element_coordinates = coordinate_map_(e);
                return are_kd_vectors_equal(target, element_coordinates);
            };

            if (is_target(*element))
                continue;

            if (!is_heap_full)
            {
                max_heap.push(element);
                continue;
            }

            element_type const* heap_root = max_heap.top();
            if (!element_less_than(element, heap_root))
                continue;

            max_heap.pop();
            max_heap.push(element);
        }

        node_type const* left_child  = current_node->left().get();
        node_type const* right_child = current_node->right().get();

        auto const dimension     = current_depth % K;
        auto const& median       = current_node->points().front();
        auto const& median_point = coordinate_map_(*median);

        aabb_type left_aabb       = current_aabb;
        left_aabb.max[dimension]  = median_point[dimension];
        aabb_type right_aabb      = current_aabb;
        right_aabb.min[dimension] = median_point[dimension];

        auto left_nearest_aabb_point  = left_aabb.nearest_point_from(target);
        auto right_nearest_aabb_point = right_aabb.nearest_point_from(target);

        auto const is_heap_full = [&]() {
            return max_heap.size() == k;
        };

        auto const visit = [&](node_type const* child,
                               aabb_type const& child_aabb,
                               coordinates_type nearest_aabb_point) {
            bool const has_child = child != nullptr;
            if (has_child)
            {
                element_type const* heap_root = max_heap.empty() ? nullptr : max_heap.top();
                bool const should_recurse =
                    !is_heap_full() ||
                    coordinates_less_than(nearest_aabb_point, coordinate_map_(*heap_root));

                if (should_recurse)
                {
                    recurse_knn<CoordinatesLessThanType, ElementLessThanType>(
                        target,
                        k,
                        child,
                        child_aabb,
                        current_depth + 1u,
                        coordinates_less_than,
                        element_less_than,
                        max_heap,
                        eps);
                }
            }
        };

        /**
         * Visit the child subtree closest to the target point first, and then
         * visit the other child.
         */
        if (common::squared_distance(left_nearest_aabb_point, target) <
            common::squared_distance(right_nearest_aabb_point, target))
        {
            visit(left_child, left_aabb, left_nearest_aabb_point);
            visit(right_child, right_aabb, right_nearest_aabb_point);
        }
        else
        {
            visit(right_child, right_aabb, right_nearest_aabb_point);
            visit(left_child, left_aabb, left_nearest_aabb_point);
        }
    }

  private:
    std::size_t max_depth_;
    std::vector<element_type> storage_;
    node_type_ptr root_;
    CoordinateMap coordinate_map_;
    kd_axis_aligned_bounding_box_t<coordinate_type, K> aabb_;
};

} // namespace pcp

#endif // PCP_KDTREE_LINKED_KDTREE_HPP
