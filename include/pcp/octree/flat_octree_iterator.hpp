#ifndef PCP_OCTREE_FLAT_OCTREE_ITERATOR_HPP
#define PCP_OCTREE_FLAT_OCTREE_ITERATOR_HPP

/**
* @file 
* @ingroup octree
*/

namespace pcp {
	
template <class Element, class ParamsType>
class basic_flat_octree_t;

/**
 * @ingroup linked-octree
 * @brief
 * Read-only forward iterator for points in the flat octree.
 * @tparam Element The element type
 * @tparam ParamsType Type containg the octree parameters
 */
template <class Element, class ParamsType>
class flat_octree_iterator_t
{
	using octree_type = basic_flat_octree_t<Element, ParamsType>;
  public:
	using element_type      = Element; ///< Type of element iterated over
    using value_type        = element_type;
    using difference_type   = std::size_t;
    using reference         = value_type&;
    using const_reference   = value_type const&;
    using pointer           = value_type*;
    using const_pointer     = value_type const*;
    using iterator_category = std::forward_iterator_tag; ///< Iterator category

  private:
    using self_type = flat_octree_iterator_t<Element, ParamsType>;
    using element_iterator = typename octree_type::element_type::iterator;
    using const_element_iterator = typename octree_type::element_type::const_iterator;
    using map_iterator = typename octree_type::map_type::iterator;
    using const_map_iterator = typename octree_type::map_type::iterator;

  public:
    friend class basic_flat_octree_t<Element, ParamsType>;

    flat_octree_iterator_t() : it_(), map_it_() {}

    flat_octree_iterator_t(octree_type::map_type* map) : it_(), map_it_() 
    { 
        map_it_         = map.begin();
        map_end_it_     = map.cend();

        if (map_it_ != map_end_it_)
        {
            current_octant_ = map_it_->second;
            it_             = current_octant_.begin();
        }
    }

    flat_octree_iterator_t(self_type const& other)     = default;
    flat_octree_iterator_t(self_type&& other) noexcept = default;

    self_type& operator=(self_type const& other) = default;

    const_reference operator*() const { return *it_; }
    reference operator*() { return *it_; }

    self_type& operator++() 
    { 
        if (++it_ != current_octant_.cend())
            return *this;

        move_to_next_octant();
        return *this;
    }

    self_type const& operator++() const { return ++(const_cast<self_type&>(*this)); }

    self_type operator++(int) 
    { 
        self_type previous{*this};
        ++(const_cast<self_type&>(*this));
        return previous;
    }

    self_type operator++(int) const { return const_cast<self_type&>(*this)++; }

    bool operator==(self_type const& other) const
    {
        return (current_octant_ == other.current_octant_) && (it_ == other.it_) &&
               (map_it_ == other.map_it_);
    }

    bool operator!=(self_type const& other) const { return !(*this == other); }

    const_pointer operator->() const { return &(*it_); }
    pointer operator->() { return &(*it_); }

  private:
    void move_to_next_octant() 
    {
        if (++map_it_ != map_end_it_)
        {
            current_octant_ = map_it_->second;
            it_             = current_octant_.begin();
        }
    }

    octree_type::container_type* current_octant_;
    element_iterator it_;
    map_iterator map_it_;
    const_map_iterator map_end_it_;
};

} // namespace pcp

#endif // PCP_OCTREE_FLAT_OCTREE_ITERATOR_HPP