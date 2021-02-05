#include <iostream>
#include <pcp/algorithm/estimate_normals.hpp>
#include <pcp/common/normals/normal.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/io/ply.hpp>
#include <pcp/octree/octree.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/transform.hpp>
#include <vector>

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage:\n"
                  << "normals-estimation.exe <input ply file> <output ply file> [k neighborhood "
                     "size] [parallel | seq]\n";
        return -1;
    }

    std::uint64_t const k                 = argc >= 4 ? std::stoull(argv[3]) : 10u;
    bool const parallel                   = argc >= 5 ? std::string(argv[4]) == "parallel" : false;
    std::filesystem::path ply_point_cloud = argv[1];

    using index_type  = std::uint64_t;
    using point_type  = pcp::point_t;
    using normal_type = pcp::normal_t;

    auto [p, n] = pcp::io::read_ply<point_type, normal_type>(ply_point_cloud);

    std::vector<point_type> points = std::move(p);
    std::vector<normal_type> normals(points.size());
    std::vector<index_type> indices = ranges::views::iota(
                                          static_cast<std::uint64_t>(0u),
                                          static_cast<std::uint64_t>(points.size())) |
                                      ranges::to<std::vector>();

    auto const index_map = [](index_type const& i) {
        return i;
    };
    auto const point_map = [&](index_type const& i) {
        return points[i];
    };
    auto const normal_map = [&](index_type const& i) {
        return normals[i];
    };

    pcp::basic_linked_octree_t<index_type> octree(indices.begin(), indices.end(), point_map);

    auto const knn = [&](index_type const& i) {
        return octree.nearest_neighbours(points[i], k, point_map);
    };

    auto const transform_op = [&](index_type const& i, pcp::normal_t const& n) {
        normals[i] = n;
        return i;
    };

    if (parallel)
    {
        pcp::algorithm::estimate_normals(
            std::execution::par,
            indices.begin(),
            indices.end(),
            indices.begin(),
            point_map,
            knn,
            transform_op);
    }
    else
    {
        pcp::algorithm::estimate_normals(
            std::execution::seq,
            indices.begin(),
            indices.end(),
            indices.begin(),
            point_map,
            knn,
            transform_op);
    }

    pcp::algorithm::propagate_normal_orientations(
        indices.begin(),
        indices.end(),
        index_map,
        knn,
        point_map,
        normal_map,
        transform_op);

    pcp::io::write_ply(
        std::filesystem::path(argv[2]),
        points,
        normals,
        pcp::io::ply_format_t::binary_little_endian);

    return 0;
}
