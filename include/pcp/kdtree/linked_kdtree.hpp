#ifndef PCP_KDTREE_LINKED_KDTREE_HPP
#define PCP_KDTREE_LINKED_KDTREE_HPP

/**
 * @file
 * @ingroup kd-tree
 */

#include "pcp/common/axis_aligned_bounding_box.hpp"
#include "pcp/common/points/point.hpp"
#include "pcp/common/vector3d_queries.hpp"
#include "pcp/kdtree/linked_kdtree_node.hpp"
#include "pcp/traits/coordinate_map.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>
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

    bool empty() const { return storage_.empty(); }
    std::size_t size() const { return storage_.size(); }
    void clear()
    {
        root_.reset();
        storage_.clear();
    }

    iterator begin() { return storage_.begin(); }
    iterator end() { return storage_.end(); }
    const_iterator cbegin() const { return storage_.cbegin(); }
    const_iterator cend() const { return storage_.cend(); }

    node_type_ptr const& root() const { return root_; }

    std::vector<element_type> nearest_neighbours(
        element_type const& element_target,
        std::size_t k,
        coordinate_type eps = static_cast<coordinate_type>(1e-5)) const
    {
        coordinates_type const& target = coordinate_map_(element_target);
        auto const distance = [this](coordinates_type const& c1, coordinates_type const& c2) {
            auto const difference = [](auto&& tup) {
                auto const& c1 = std::get<0>(tup);
                auto const& c2 = std::get<1>(tup);
                return c2 - c1;
            };

            auto const rng = ranges::views::zip(c1, c2) | ranges::views::transform(difference);

            coordinates_type c1_to_c2{};
            std::copy(rng.begin(), rng.end(), c1_to_c2.begin());

            auto const distance = std::inner_product(
                c1_to_c2.begin(),
                c1_to_c2.end(),
                c1_to_c2.begin(),
                coordinate_type{0});

            return distance;
        };

        auto const less_than_coordinates =
            [eps, target, distance](coordinates_type const& c1, coordinates_type const& c2) {
                auto const distance1 = distance(target, c1);
                auto const distance2 = distance(target, c2);

                return distance1 < distance2 &&
                       !common::floating_point_equals(distance1, distance2, eps);
            };

        auto const less_than_elements =
            [target, less_than_coordinates, this](element_type const* e1, element_type const* e2) {
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
            distance);

        std::vector<element_type> knearest_neighbours{};
        knearest_neighbours.reserve(k);
        while (!max_heap.empty())
        {
            knearest_neighbours.push_back(*max_heap.top());
            max_heap.pop();
        }
        return knearest_neighbours;
    }

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
            auto begin     = storage_.begin() + first;
            auto end       = storage_.begin() + last + 1u;
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

        auto begin = storage_.begin() + first;
        auto end   = storage_.begin() + last + 1u;

        bool const parallelize = size >= min_element_count_for_parallel_exec;
        if (parallelize)
            std::nth_element(std::execution::par, begin, begin + size / 2u, end, less_than);
        else
            std::nth_element(std::execution::seq, begin, begin + size / 2u, end, less_than);

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

    std::unique_ptr<node_type>
    construct_presort_recursive(std::size_t first, std::size_t last, std::size_t current_depth)
    {
    }

    template <class CoordinatesLessThanType, class ElementLessThanType, class DistanceFunctionType>
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
        DistanceFunctionType const& distance) const
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
                auto heap_root             = max_heap.top();
                auto heap_root_coordinates = coordinate_map_(*heap_root);
                bool const should_recurse =
                    !is_heap_full() ||
                    coordinates_less_than(nearest_aabb_point, heap_root_coordinates);

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
                        distance);
                }
            }
        };

        /**
         * Visit the child subtree closest to the target point first, and then
         * visit the other child.
         */
        if (distance(left_nearest_aabb_point, target) < distance(right_nearest_aabb_point, target))
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
