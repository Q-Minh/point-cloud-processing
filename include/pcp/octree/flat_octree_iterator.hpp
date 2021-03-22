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
    using self_type                = flat_octree_iterator_t<Element, ParamsType>;
    using octree_container_type    = typename octree_type::container_type;
    using octree_map_type          = typename octree_type::map_type;
    using container_iterator       = typename octree_container_type::iterator;
    using const_container_iterator = typename octree_container_type::const_iterator;
    using map_iterator             = typename octree_map_type::iterator;
    using const_map_iterator       = typename octree_map_type::const_iterator;

  public:
    friend class basic_flat_octree_t<Element, ParamsType>;

    flat_octree_iterator_t(octree_map_type* map)
        : map_it_(map->begin()), map_end_it_(map->end()), it_()
    {
        refresh();
    }

    flat_octree_iterator_t(map_iterator begin, map_iterator end, container_iterator it)
        : map_it_(begin), map_end_it_(end), it_(it)
    {
    }

    flat_octree_iterator_t(self_type const& other)     = default;
    flat_octree_iterator_t(self_type&& other) noexcept = default;

    self_type& operator=(self_type const& other) = default;

    const_reference operator*() const { return *it_; }
    reference operator*() { return *it_; }

    self_type& operator++()
    {
        auto& current_octant = map_it_->second;
        if (++it_ != current_octant.cend())
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
        return (map_it_ == other.map_it_) && (map_it_ == map_end_it_ || it_ == other.it_);
    }

    bool operator!=(self_type const& other) const { return !(*this == other); }

    const_pointer operator->() const { return &(*it_); }
    pointer operator->() { return &(*it_); }

    void make_end_iterator() { map_it_ = map_end_it_; }

  private:
    void refresh()
    {
        if (map_it_ == map_end_it_)
            return;

        auto& current_octant = map_it_->second;
        it_                  = current_octant.begin();
    }

    void move_to_next_octant()
    {
        if (++map_it_ == map_end_it_)
            return;

        auto& current_octant = map_it_->second;
        it_                  = current_octant.begin();
    }

    map_iterator map_it_;
    map_iterator map_end_it_;
    container_iterator it_;
};

} // namespace pcp

#endif // PCP_OCTREE_FLAT_OCTREE_ITERATOR_HPP