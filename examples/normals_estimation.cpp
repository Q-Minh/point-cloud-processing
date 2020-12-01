#include <iostream>
#include <pcp/algorithm/common.hpp>
#include <pcp/algorithm/estimate_normals.hpp>
#include <pcp/common/normals/normal.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/common/points/vertex.hpp>
#include <pcp/io/ply.hpp>
#include <pcp/octree/octree.hpp>

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr
            << "Usage:\n"
            << "normals-estimation.exe <input ply file> <output ply file> [k neighborhood size]\n";
        return -1;
    }

    std::uint64_t const k                 = argc == 4 ? std::stoull(argv[3]) : 10u;
    std::filesystem::path ply_point_cloud = argv[1];

    using point_type  = pcp::point_t;
    using normal_type = pcp::normal_t;
    using vertex_type = pcp::vertex_t;

    auto [points, normals] = pcp::io::read_ply<point_type, normal_type>(ply_point_cloud);

    std::vector<vertex_type> vertices;
    vertices.reserve(points.size());

    for (std::size_t i = 0; i < points.size(); ++i)
        vertices.push_back(vertex_type{&points[i], i});

    normals.resize(points.size());

    using iterator_type = typename decltype(points)::const_iterator;
    auto const bounding_box =
        pcp::bounding_box<iterator_type, pcp::point_t>(std::cbegin(points), std::cend(points));

    auto nx = bounding_box.max.x() - bounding_box.min.x();
    auto ny = bounding_box.max.y() - bounding_box.min.y();
    auto nz = bounding_box.max.z() - bounding_box.min.z();

    auto const xpow = static_cast<int>(std::log2f(nx));
    auto const ypow = static_cast<int>(std::log2f(ny));
    auto const zpow = static_cast<int>(std::log2f(nz));

    auto const maxpow = std::max(std::max(xpow, ypow), zpow);
    auto const n      = static_cast<float>(std::pow(2.f, maxpow + 1));

    pcp::octree_parameters_t<pcp::point_t> params;
    params.voxel_grid = {{-n, -n, -n}, {n, n, n}};

    pcp::basic_octree_t<vertex_type, decltype(params)> octree{
        std::cbegin(vertices),
        std::cend(vertices),
        params};

    auto const knn = [&octree, &vertices](vertex_type const& v) {
        std::uint64_t const k = 15u;
        return octree.nearest_neighbours(vertices[v.id()], k);
    };
    auto const get_point = [&vertices](vertex_type const& v) {
        return pcp::point_t{vertices[v.id()]};
    };
    auto const get_normal = [normals_ptr = &normals](vertex_type const& v) {
        auto const& normals = *normals_ptr;
        return normals[v.id()];
    };
    auto const transform_op = [normals_ptr =
                                   &normals](vertex_type const& v, pcp::normal_t const& n) {
        auto& normals   = *normals_ptr;
        normals[v.id()] = n;
    };

    pcp::algorithm::estimate_normals(
        vertices.begin(),
        vertices.end(),
        normals.begin(),
        knn,
        pcp::algorithm::default_normal_transform<vertex_type, normal_type>);

    pcp::algorithm::propagate_normal_orientations(
        vertices.begin(),
        vertices.end(),
        knn,
        get_point,
        get_normal,
        transform_op);

    pcp::io::write_ply(
        std::filesystem::path(argv[2]),
        points,
        normals,
        pcp::io::ply_format_t::binary_little_endian);

    return 0;
}
