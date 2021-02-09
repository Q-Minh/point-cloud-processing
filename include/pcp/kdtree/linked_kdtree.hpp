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

        /**
         * State tracking variables
         */
        std::size_t current_depth{0u};
        node_type const* current_node = root_.get();
        std::priority_queue<element_type*, std::vector<element_type*>, decltype(less_than_elements)>
            max_heap(less_than_elements);

        /**
         * Downwards pass:
         * Find the leaf node in which we would have inserted the point `target`, to find
         * a best first estimate of where the nearest neighbours should be found.
         */
        while (current_node->is_internal())
        {
            auto const dimension     = current_depth % K;
            auto const& median       = current_node->points().front();
            auto const& median_point = coordinate_map_(*median);

            if (target[dimension] >= median_point[dimension])
            {
                current_node = current_node->right().get();
            }
            if (target[dimension] < median_point[dimension])
            {
                current_node = current_node->left().get();
            }
        }

        // Initialize max heap with best initial guesses
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
            if (!less_than_elements(element, heap_root))
                continue;

            max_heap.pop();
            max_heap.push(element);
        }

        // recursive tree traversal
        node_type const* downwards_pass_node = current_node;
        current_node                         = root_.get();
        recurse_knn<decltype(less_than_coordinates), decltype(less_than_elements)>(
            target,
            k,
            current_node,
            aabb_,
            0u,
            less_than_coordinates,
            less_than_elements,
            max_heap,
            downwards_pass_node,
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
    void construct_presort()
    {
        // std::sort(storage_.begin(), storage_.end(), less_than_t{coordinate_map_});
        // std::size_t first         = 0u;
        // std::size_t last          = storage_.size() - 1u;
        // std::size_t current_depth = 0u;
        // root_                     = construct_presort_recursive(first, last, current_depth);
    }

    std::unique_ptr<node_type>
    construct_presort_recursive(std::size_t first, std::size_t last, std::size_t current_depth)
    {
        ///**
        // * No left sub-tree for parent node
        // */
        // if (last < first)
        //{
        //    return nullptr;
        //}

        // auto node    = std::make_unique<node_type>();
        // auto& points = node->points();

        ///**
        // * Leaf node
        // */
        // if (current_depth == max_depth_ - 1u)
        //{
        //    points.resize((last + 1u) - first);
        //    auto begin     = storage_.begin() + first;
        //    auto end       = storage_.begin() + last + 1u;
        //    auto out_begin = points.begin();

        //    std::transform(begin, end, out_begin, [](element_type& e) {
        //        return std::addressof(e);
        //    });
        //    return node;
        //}

        ///**
        // * Leaf node
        // */
        // if (first == last)
        //{
        //    points.push_back(std::addressof(storage_[first]));
        //    return node;
        //}

        //// 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

        //// root: (0 + 9) / 2 = 4
        ////       left=(0, 3) --- right=(5,9)

        //// left recurse: (0 + 3) / 2 = 1
        ////               left=(0,0) --- right=(2,3)

        //// right recurse: (5 + 9) / 2 = 7
        ////                left=(5, 6) ---- right=(8,9)

        //// left-left recurse: end condition

        //// left-right recurse: (2 + 3) / 2 = 2
        ////                     left=null --- right=(3,3)

        //// right-left recurse: (5 + 6) / 2 = 5
        ////                     left=null --- right=(6,6)

        //// right-right recurse: (8 + 9) / 2 = 8
        ////                      left=null --- right=(9,9)

        // auto median = (first + last) / 2u;
        // points.push_back(std::addressof(storage_[median]));

        // auto left_child  = construct_presort_recursive(first, median - 1u, current_depth + 1u);
        // auto right_child = construct_presort_recursive(median + 1u, last, current_depth + 1u);
        // node->set_left(std::move(left_child));
        // node->set_right(std::move(right_child));

        // return node;
    }

    template <class LessThanCoordinatesType, class LessThanElementsType, class DistanceFunctionType>
    void recurse_knn(
        coordinates_type const& target,
        std::size_t k,
        node_type const* current_node,
        aabb_type const& current_aabb,
        std::size_t current_depth,
        LessThanCoordinatesType const& less_than_coordinates,
        LessThanElementsType const& less_than_elements,
        std::priority_queue<element_type*, std::vector<element_type*>, LessThanElementsType>&
            max_heap,
        node_type const* downwards_pass_node,
        DistanceFunctionType const& distance) const
    {
        // Don't revisit the initial node
        if (current_node == downwards_pass_node)
            return;

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
            if (!less_than_elements(element, heap_root))
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

        auto const visit_left = [&]() {
            bool const has_left_child = left_child != nullptr;
            if (has_left_child)
            {
                auto heap_root        = max_heap.top();
                auto root_coordinates = coordinate_map_(*heap_root);
                bool const should_recurse_left =
                    max_heap.size() < k ||
                    less_than_coordinates(left_nearest_aabb_point, root_coordinates);
                if (should_recurse_left)
                {
                    recurse_knn<LessThanCoordinatesType, LessThanElementsType>(
                        target,
                        k,
                        left_child,
                        left_aabb,
                        current_depth + 1u,
                        less_than_coordinates,
                        less_than_elements,
                        max_heap,
                        downwards_pass_node,
                        distance);
                }
            }
        };

        auto const visit_right = [&]() {
            bool const has_right_child = right_child != nullptr;
            if (has_right_child)
            {
                auto heap_root        = max_heap.top();
                auto root_coordinates = coordinate_map_(*heap_root);
                bool const should_recurse_right =
                    max_heap.size() < k ||
                    less_than_coordinates(right_nearest_aabb_point, root_coordinates);
                if (should_recurse_right)
                {
                    recurse_knn<LessThanCoordinatesType, LessThanElementsType>(
                        target,
                        k,
                        right_child,
                        right_aabb,
                        current_depth + 1u,
                        less_than_coordinates,
                        less_than_elements,
                        max_heap,
                        downwards_pass_node,
                        distance);
                }
            }
        };

        if (distance(left_nearest_aabb_point, target) < distance(right_nearest_aabb_point, target))
        {
            visit_left();
            visit_right();
        }
        else
        {
            visit_right();
            visit_left();
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
