#ifndef PCP_basic_kdtree_t_HPP
#define PCP_basic_kdtree_t_HPP

/**
 * @file
 * @ingroup kd-tree
 */

#include "pcp/common/points/point.hpp"
#include "pcp/kdtree/linked_kdtree_node.hpp"
#include "pcp/traits/coordinate_map.hpp"

#include <algorithm>

namespace pcp {
namespace kdtree {
enum class construction_t { nth_element, presort };
}

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
        CoordinateMap coordinate_map = CoordinateMap{},
        std::size_t max_depth        = 12u,
        kdtree::construction_t c     = kdtree::construction_t::nth_element)
        : max_depth_{max_depth}, storage_(begin, end), root_{}, coordinate_map_{coordinate_map}
    {
        if (c == kdtree::construction_t::nth_element)
        {
            construct_nth_element();
        }
        if (c == kdtree::construction_t::presort)
        {
            // TODO: Perform correct presorting in exact median construction method
            // construct_presort();
        }
    }

    bool empty() const { return storage_.empty(); }
    std::size_t size() const { return storage_.size(); }

    iterator begin() { return storage_.begin(); }
    iterator end() { return storage_.end(); }
    const_iterator cbegin() const { return storage_.cbegin(); }
    const_iterator cend() const { return storage_.cend(); }

    node_type_ptr const& root() const { return root_; }

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
    void construct_nth_element()
    {
        auto size = storage_.size();
        root_     = construct_nth_element_recursive(0u, size - 1u, 0u);
    }

    std::unique_ptr<node_type> construct_nth_element_recursive(
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
        std::nth_element(begin, begin + size / 2u, end, less_than);
        auto median = first + size / 2u;
        points.push_back(std::addressof(storage_[median]));

        ++current_depth;
        auto left_child = construct_nth_element_recursive(first, median - 1u, current_depth);
        auto right_child = construct_nth_element_recursive(median + 1u, last, current_depth);
        node->set_left(std::move(left_child));
        node->set_right(std::move(right_child));

        return node;
    }

    /**
     * @brief Do not use.
     */
    void construct_presort()
    {
        //std::sort(storage_.begin(), storage_.end(), less_than_t{coordinate_map_});
        //std::size_t first         = 0u;
        //std::size_t last          = storage_.size() - 1u;
        //std::size_t current_depth = 0u;
        //root_                     = construct_presort_recursive(first, last, current_depth);
    }

    std::unique_ptr<node_type>
    construct_presort_recursive(std::size_t first, std::size_t last, std::size_t current_depth)
    {
        ///**
        // * No left sub-tree for parent node
        // */
        //if (last < first)
        //{
        //    return nullptr;
        //}

        //auto node    = std::make_unique<node_type>();
        //auto& points = node->points();

        ///**
        // * Leaf node
        // */
        //if (current_depth == max_depth_ - 1u)
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
        //if (first == last)
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

        //auto median = (first + last) / 2u;
        //points.push_back(std::addressof(storage_[median]));

        //auto left_child  = construct_presort_recursive(first, median - 1u, current_depth + 1u);
        //auto right_child = construct_presort_recursive(median + 1u, last, current_depth + 1u);
        //node->set_left(std::move(left_child));
        //node->set_right(std::move(right_child));

        //return node;
    }

  private:
    std::size_t max_depth_;
    std::vector<element_type> storage_;
    node_type_ptr root_;
    CoordinateMap coordinate_map_;
};

} // namespace pcp

#endif // PCP_basic_kdtree_t_HPP
