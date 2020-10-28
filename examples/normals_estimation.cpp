#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <pcp/io/ply.hpp>
#include <pcp/normal.hpp>
#include <pcp/octree.hpp>
#include <pcp/point.hpp>
#include <pcp/point_view.hpp>

struct vertex_t
{
    using coordinate_type = typename pcp::point_t::coordinate_type;
    using component_type  = coordinate_type;
    using T               = typename pcp::point_t::coordinate_type;
    using self_type       = vertex_t;

    vertex_t()                       = default;
    vertex_t(self_type const& other) = default;
    vertex_t(self_type&& other)      = default;
    self_type& operator=(self_type const& other) = default;
    self_type& operator=(self_type&& other) = default;
    explicit vertex_t(pcp::point_t other) : point_(&other), normal_(nullptr) {}
    explicit vertex_t(pcp::point_t* other) : point_(other), normal_(nullptr) {}
    explicit vertex_t(pcp::point_t* p, pcp::normal_t* n) : point_(p), normal_(n) {}

    void point(pcp::point_t* p) { point_ = p; }
    void normal(pcp::normal_t* n) { normal_ = n; }

    T const& x() const { return point_->x(); }
    T const& y() const { return point_->y(); }
    T const& z() const { return point_->z(); }
    T const& nx() const { return normal_->x(); }
    T const& ny() const { return normal_->y(); }
    T const& nz() const { return normal_->z(); }

    void x(T value) { point_->x(value); }
    void y(T value) { point_->y(value); }
    void z(T value) { point_->z(value); }
    void nx(T value) { normal_->x(value); }
    void ny(T value) { normal_->y(value); }
    void nz(T value) { normal_->z(value); }

    template <class PointView>
    bool operator==(PointView const& other) const
    {
        T constexpr e     = static_cast<T>(1e-5);
        T const dx        = std::abs(x() - other.x());
        T const dy        = std::abs(y() - other.y());
        T const dz        = std::abs(z() - other.z());
        bool const equals = (dx < e) && (dy < e) && (dz < e);
        return equals;
    }
    template <class PointView>
    bool operator!=(PointView const& other) const
    {
        return !(*this == other);
    }

  private:
    pcp::point_t* point_;
    pcp::normal_t* normal_;
};

int main(int argc, char** argv)
{
    std::filesystem::path ply_point_cloud =
        "../../../examples/data/stanford_bunny.ply" /* argv[1] */;

    auto [points, normals] = pcp::io::read_ply<pcp::point_t, pcp::normal_t>(ply_point_cloud);
    normals.resize(points.size());

    std::vector<vertex_t> vertices;
    vertices.reserve(points.size());

    for (std::size_t i = 0; i < points.size(); ++i)
        vertices.emplace_back(&points[i], &normals[i]);

    using iterator_type = typename decltype(points)::const_iterator;
    auto const voxel_grid =
        pcp::bounding_box<iterator_type, pcp::point_t>(std::cbegin(points), std::cend(points));
    pcp::octree_parameters_t<pcp::point_t> params;
    params.voxel_grid = voxel_grid;

    pcp::basic_octree_t<vertex_t, decltype(params)> octree{
        std::cbegin(vertices),
        std::cend(vertices),
        params};

    std::size_t k = 5u;

    for (vertex_t& vertex : vertices)
    {
        auto const knn = octree.nearest_neighbours(vertex, k);

        Eigen::Matrix3Xf V;
        V.resize(3, knn.size());
        for (int i = 0; i < knn.size(); ++i)
            V.block(0, i, 3, 1) = Eigen::Vector3f(knn[i].x(), knn[i].y(), knn[i].z());

        Eigen::Vector3f const Mu      = V.rowwise().mean();
        Eigen::Matrix3Xf const Vprime = V.colwise() - Mu;
        Eigen::Matrix3f const Cov     = Vprime * Vprime.transpose();
        Eigen::SelfAdjointEigenSolver<decltype(Cov)> A(Cov);
        auto const l = A.eigenvalues();
        auto const& q = A.eigenvectors();

        pcp::normal_t normal{};
        // instead of sorting, just use 3 if statements
        // First eigenvalue is smallest
        if (l(0) <= l(1) && l(0) <= l(2))
        {
            normal = {q(0, 0), q(1, 0), q(2, 0)};
        }
        // Second eigenvalue is smallest
        if (l(1) <= l(0) && l(1) <= l(2))
        {
            normal = {q(0, 1), q(1, 1), q(2, 1)};
        }
        // Third eigenvalue is smallest
        if (l(2) <= l(0) && l(2) <= l(1))
        {
            normal = {q(0, 2), q(1, 2), q(2, 2)};
        }

        vertex.nx(normal.x());
        vertex.ny(normal.y());
        vertex.nz(normal.z());
    }

    pcp::io::write_ply(
        std::filesystem::path("../../../examples/data/stanford_bunny_normals.ply" /* argv[2] */),
        points,
        normals,
        pcp::io::ply_format_t::binary_little_endian);

    return 0;
}
