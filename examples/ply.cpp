#include <iostream>
#include <pcp/common/normals/normal.hpp>
#include <pcp/common/points/point.hpp>
#include <pcp/io/ply.hpp>

int main(int argc, char** argv)
{
    if (argc != 4)
    {
        std::cerr << "Usage:\n"
                  << "./pcp-ply.exe <input ply file path> <output ply file path> <format>\n\n"
                  << "\t'format'\tascii | binary_little_endian | binary_big_endian\n";

        return 0;
    }

    std::filesystem::path ply_point_cloud = argv[1];
    auto [points, normals, colors] =
        pcp::io::read_ply<pcp::point_t, pcp::normal_t>(ply_point_cloud);

    std::cout << "Read ply point cloud\n"
              << "Vertices: " << std::to_string(points.size()) << "\n"
              << "Normals: " << std::to_string(normals.size()) << "\n"
              << "Colors: " << std::to_string(colors.size()) << "\n";

    std::cout << "Writing to " << argv[2] << "\n";

    std::filesystem::path out = argv[2];
    pcp::io::write_ply(out, points, normals, colors, pcp::io::string_to_format(argv[3]));

    return 0;
}