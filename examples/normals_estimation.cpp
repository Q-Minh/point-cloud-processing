#include <iostream>
#include <pcp/algorithm/algorithm.hpp>
#include <pcp/common/normals/normal.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/io/ply.hpp>
#include <pcp/kdtree/kdtree.hpp>
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
                     "size] [parallel | seq] [bilateral <k iterations> <sigmaf multiplier> <sigmag "
                     "multiplier>]\n";
        return -1;
    }

    std::uint64_t const k = argc >= 4 ? std::stoull(argv[3]) : 10u;
    bool const parallel   = argc >= 5 ? std::string(argv[4]) == "parallel" : false;
    bool const bilateral  = argc >= 6 ? std::string(argv[5]) == "bilateral" : false;

    if (bilateral && argc < 9)
    {
        std::cerr << "bilateral argument requires:\n"
                  << "\tk iterations: the number of iterations of bilateral filtering to apply\n"
                  << "\tsigmaf multiplier: multiplier of average point distance to neighbors "
                     "computing sigmaf = multiplier * avg\n"
                  << "\tsigmag multiplier: multiplier of average point distance to neighbors "
                     "computing sigmag = multiplier * avg\n";
        return -1;
    }

    std::size_t const bk                  = std::stoull(argv[6]);
    double const sigmaf_multiplier        = std::stod(argv[7]);
    double const sigmag_multiplier        = std::stod(argv[8]);
    std::filesystem::path ply_point_cloud = argv[1];

    using index_type  = std::uint64_t;
    using point_type  = pcp::point_t;
    using normal_type = pcp::normal_t;

    auto [p, n, _] = pcp::io::read_ply<point_type, normal_type>(ply_point_cloud);

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
    auto const coordinate_map = [&](index_type const& i) {
        return std::array<float, 3u>{points[i].x(), points[i].y(), points[i].z()};
    };

    pcp::kdtree::construction_params_t params;
    params.compute_max_depth = true;

    pcp::basic_linked_kdtree_t<index_type, 3u, decltype(coordinate_map)> kdtree{
        indices.begin(),
        indices.end(),
        coordinate_map,
        params};

    auto const knn = [&](index_type const& i) {
        return kdtree.nearest_neighbours(i, k);
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

    if (bilateral)
    {
        float avg = pcp::algorithm::average_distance_to_neighbors(
            std::execution::par,
            indices.begin(),
            indices.end(),
            point_map,
            knn);

        pcp::algorithm::bilateral::params_t params;
        params.K      = bk;
        params.sigmaf = sigmaf_multiplier * static_cast<double>(avg);
        params.sigmag = sigmag_multiplier * static_cast<double>(avg);

        pcp::algorithm::bilateral_filter_normals(
            std::execution::par,
            indices.begin(),
            indices.end(),
            normals.begin(),
            point_map,
            normal_map,
            params);
    }

    pcp::io::write_ply(
        std::filesystem::path(argv[2]),
        points,
        normals,
        _,
        pcp::io::ply_format_t::binary_little_endian);

    return 0;
}
