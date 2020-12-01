#pragma once

#include "endianness.hpp"
#include "pcp/traits/normal_traits.hpp"
#include "pcp/traits/point_traits.hpp"
#include "pcp/traits/triangle_traits.hpp"
#include "tokenize.hpp"

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace pcp {
namespace io {

enum class ply_format_t { ascii, binary_little_endian, binary_big_endian };

enum class ply_coordinate_type_t { single_precision, double_precision };

struct ply_parameters_t
{
    ply_format_t format                         = ply_format_t::ascii;
    std::size_t vertex_count                    = 0u;
    std::size_t normal_count                    = 0u;
    ply_coordinate_type_t vertex_component_type = ply_coordinate_type_t::single_precision;
    ply_coordinate_type_t normal_component_type = ply_coordinate_type_t::single_precision;
};

inline ply_format_t string_to_format(std::string const& s)
{
    ply_format_t format = ply_format_t::ascii;
    if (s == "ascii")
        format = ply_format_t::ascii;
    if (s == "binary_little_endian")
        format = ply_format_t::binary_little_endian;
    if (s == "binary_big_endian")
        format = ply_format_t::binary_big_endian;
    return format;
}

template <class Point, class Normal>
inline auto read_ply(std::istream& is) -> std::tuple<std::vector<Point>, std::vector<Normal>>;

template <class Point, class Normal>
inline auto read_ply_ascii(std::istream& is, ply_parameters_t const& params)
    -> std::tuple<std::vector<Point>, std::vector<Normal>>;

template <class Point, class Normal>
inline auto read_ply_binary(std::istream& is, ply_parameters_t const& params)
    -> std::tuple<std::vector<Point>, std::vector<Normal>>;

template <class Point, class Normal>
inline auto read_ply_binary_little_endian(std::istream& is, ply_parameters_t const& params)
    -> std::tuple<std::vector<Point>, std::vector<Normal>>;

template <class Point, class Normal>
inline auto read_ply_binary_big_endian(std::istream& is, ply_parameters_t const& params)
    -> std::tuple<std::vector<Point>, std::vector<Normal>>;

/**
 * @brief
 * Reads a ply file into a point cloud (potentially with normals).
 * The function supports ascii, binary little endian, binary big endian.
 * The ply file must have x,y,z properties and nx,ny,nz properties if
 * normals are available and ignores all other properties. The properties
 * must be floats.
 * @tparam Point Type of the point cloud's points to return.
 * @tparam Normal Type of the point cloud's normals to return.
 * @param is The input ply file stream.
 * @return Returns the point cloud as a tuple of vector of points and vector of normals.
 */
template <class Point, class Normal>
inline auto read_ply(std::filesystem::path const& path)
    -> std::tuple<std::vector<Point>, std::vector<Normal>>
{
    if (!path.has_filename())
        return {};

    if (!path.has_extension() || path.extension() != ".ply")
        return {};

    if (!std::filesystem::exists(path))
        return {};

    std::ifstream fs{path.string(), std::ios::binary};

    if (!fs.is_open())
        return {};

    return read_ply<Point, Normal>(fs);
}

template <class Point, class Normal>
inline auto read_ply(std::istream& is) -> std::tuple<std::vector<Point>, std::vector<Normal>>
{
    static_assert(traits::is_point_v<Point>, "Point must satisfy Point concept");
    static_assert(traits::is_normal_v<Normal>, "Normal must satisfy Normal concept");

    ply_parameters_t ply_params;

    auto const string_to_coordinate_type = [](std::string const& s) -> ply_coordinate_type_t {
        ply_coordinate_type_t type = ply_coordinate_type_t::single_precision;
        if (s == "float")
            type = ply_coordinate_type_t::single_precision;
        if (s == "double")
            type = ply_coordinate_type_t::double_precision;
        return type;
    };

    auto const has_valid_vertex_properties = [string_to_coordinate_type, &is](
                                                 std::array<std::string, 3> const& property_names,
                                                 ply_coordinate_type_t& coordinate_type) -> bool {
        std::array<std::string, 3> nxyz;
        std::array<ply_coordinate_type_t, 3> types;
        std::string line;
        for (std::uint8_t i = 0; i < 3u; ++i)
        {
            std::getline(is, line);
            if (is.bad())
                return false;

            auto const tokens = tokenize(line);
            if (tokens.front() != "property")
                return false;

            if (tokens.at(1) == "list")
                return false;

            types[i]        = string_to_coordinate_type(tokens.at(1));
            coordinate_type = types[i];
            nxyz[i]         = tokens.back();
        }

        bool const are_components_of_same_type = std::all_of(
            std::cbegin(types),
            std::cend(types),
            [coordinate_type](ply_coordinate_type_t const& t) { return coordinate_type == t; });

        bool const are_components_well_named = nxyz[0] == property_names[0] &&
                                               nxyz[1] == property_names[1] &&
                                               nxyz[2] == property_names[2];

        return are_components_of_same_type && are_components_well_named;
    };

    std::string line;
    bool is_ply = false;

    /**
     * Read ply header
     */
    while (std::getline(is, line))
    {
        auto const tokens = tokenize(line);

        if (!is_ply)
        {
            if (tokens.front() != "ply")
                return {};

            is_ply = true;
            continue;
        }

        if (tokens.empty())
            continue;

        if (tokens.front() == "comment")
            continue;

        if (tokens.front() == "format")
        {
            ply_params.format = string_to_format(tokens.at(1));
            // ignore version
            continue;
        }

        if (tokens.front() == "element" && tokens.at(1) == "vertex")
        {
            ply_params.vertex_count = std::stoull(tokens.back());

            bool const is_vertex_valid =
                has_valid_vertex_properties({"x", "y", "z"}, ply_params.vertex_component_type);

            if (!is_vertex_valid)
                return {};

            continue;
        }

        if (tokens.front() == "element" && tokens.at(1) == "normal")
        {
            ply_params.normal_count = std::stoull(tokens.back());

            bool const is_normal_valid =
                has_valid_vertex_properties({"nx", "ny", "nz"}, ply_params.normal_component_type);

            if (!is_normal_valid)
                return {};

            continue;
        }

        if (tokens.front() == "end_header")
            break;
    }

    switch (ply_params.format)
    {
        case ply_format_t::ascii: return read_ply_ascii<Point, Normal>(is, ply_params);
        case ply_format_t::binary_little_endian:
            return read_ply_binary_little_endian<Point, Normal>(is, ply_params);
        case ply_format_t::binary_big_endian:
            return read_ply_binary_big_endian<Point, Normal>(is, ply_params);
        default: return {};
    }
}

/**
 * @brief
 * Writes the given point cloud to a ply file in the ply
 * format of choice at the location given by filepath.
 * @tparam Point The point cloud's point type
 * @tparam Normal The point cloud's normal type
 * @param filepath Path to the ply file to write to
 * @param vertices The points
 * @param normals The normals
 * @param format The desired ply format with which to write
 */
template <class Point, class Normal, std::enable_if_t<traits::is_normal_v<Normal>, int> = 0>
inline void write_ply(
    std::filesystem::path const& filepath,
    std::vector<Point> const& vertices,
    std::vector<Normal> const& normals,
    ply_format_t format = ply_format_t::ascii)
{
    if (!filepath.has_extension() || filepath.extension() != ".ply")
        return;

    if (vertices.empty())
        return;

    std::ofstream ofs{filepath.c_str(), std::ios::binary};

    if (!ofs.is_open())
        return;

    write_ply<Point, Normal>(ofs, vertices, normals, format);
}

template <class Point, class Normal, std::enable_if_t<traits::is_normal_v<Normal>, int> = 0>
inline void write_ply(
    std::ostream& os,
    std::vector<Point> const& vertices,
    std::vector<Normal> const& normals,
    ply_format_t format = ply_format_t::ascii)
{
    using point_type            = Point;
    using normal_type           = Normal;
    using vertex_component_type = typename Point::coordinate_type;
    using normal_component_type = typename Normal::component_type;

    std::string const vertex_component_type_str =
        std::is_same_v<vertex_component_type, double> ? "double" : "float";

    std::string const normal_component_type_str =
        std::is_same_v<normal_component_type, double> ? "double" : "float";

    std::ostringstream header_stream{};
    header_stream << "ply\n";

    if (format == ply_format_t::ascii)
        header_stream << "format ascii 1.0\n";
    if (format == ply_format_t::binary_little_endian)
        header_stream << "format binary_little_endian 1.0\n";
    if (format == ply_format_t::binary_big_endian)
        header_stream << "format binary_big_endian 1.0\n";

    header_stream << "element vertex " << vertices.size() << "\n"
                  << "property " << vertex_component_type_str << " x\n"
                  << "property " << vertex_component_type_str << " y\n"
                  << "property " << vertex_component_type_str << " z\n"
                  << "element normal " << normals.size() << "\n"
                  << "property " << normal_component_type_str << " nx\n"
                  << "property " << normal_component_type_str << " ny\n"
                  << "property " << normal_component_type_str << " nz\n"
                  << "end_header\n";

    std::string const header = header_stream.str();
    os << header;

    if (format == ply_format_t::ascii)
    {
        for (point_type const& v : vertices)
        {
            std::ostringstream oss{};
            oss << std::to_string(v.x()) << " " << std::to_string(v.y()) << " "
                << std::to_string(v.z()) << "\n";
            os << oss.str();
        }
        for (normal_type const& n : normals)
        {
            std::ostringstream oss{};
            oss << std::to_string(n.nx()) << " " << std::to_string(n.ny()) << " "
                << std::to_string(n.nz()) << "\n";
            os << oss.str();
        }
    }

    // Note: disk I/O writes can be parallelized
    auto const write_binary_data =
        [&os](std::vector<point_type> const& p, std::vector<normal_type> const& n) {
            for (std::size_t i = 0u; i < p.size(); ++i)
            {
                auto constexpr size_of_vertex_component_type = sizeof(vertex_component_type);
                std::array<std::byte, 3u * size_of_vertex_component_type> vertex_storage;

                std::byte* const data = vertex_storage.data();

                vertex_component_type* const x = reinterpret_cast<float*>(data);
                vertex_component_type* const y =
                    reinterpret_cast<float*>(data + (1u * size_of_vertex_component_type));
                vertex_component_type* const z =
                    reinterpret_cast<float*>(data + (2u * size_of_vertex_component_type));
                *x = p[i].x();
                *y = p[i].y();
                *z = p[i].z();

                os.write(
                    reinterpret_cast<const char*>(data),
                    static_cast<std::streamsize>(vertex_storage.size()));
            }

            for (std::size_t i = 0u; i < n.size(); ++i)
            {
                auto constexpr size_of_normal_component_type = sizeof(normal_component_type);
                std::array<std::byte, 3u * size_of_normal_component_type> normal_storage;

                std::byte* const data = normal_storage.data();

                normal_component_type* const nx = reinterpret_cast<float*>(data);
                normal_component_type* const ny =
                    reinterpret_cast<float*>(data + (1u * size_of_normal_component_type));
                normal_component_type* const nz =
                    reinterpret_cast<float*>(data + (2u * size_of_normal_component_type));
                *nx = n[i].nx();
                *ny = n[i].ny();
                *nz = n[i].nz();

                os.write(
                    reinterpret_cast<const char*>(data),
                    static_cast<std::streamsize>(normal_storage.size()));
            }
        };

    auto const transform_endianness = [](std::vector<point_type>& p, std::vector<normal_type>& n) {
        std::transform(std::begin(p), std::end(p), std::begin(p), [](point_type const& point) {
            return point_type{
                reverse_endianness(point.x()),
                reverse_endianness(point.y()),
                reverse_endianness(point.z())};
        });
        std::transform(std::begin(n), std::end(n), std::begin(n), [](normal_type const& normal) {
            return normal_type{
                reverse_endianness(normal.nx()),
                reverse_endianness(normal.ny()),
                reverse_endianness(normal.nz())};
        });
    };

    if (format == ply_format_t::binary_little_endian)
    {
        if (!is_machine_little_endian())
        {
            auto endian_correct_vertices = vertices;
            auto endian_correct_normals  = normals;
            transform_endianness(endian_correct_vertices, endian_correct_normals);
            write_binary_data(endian_correct_vertices, endian_correct_normals);
        }
        else
        {
            write_binary_data(vertices, normals);
        }
    }

    if (format == ply_format_t::binary_big_endian)
    {
        if (!is_machine_big_endian())
        {
            auto endian_correct_vertices = vertices;
            auto endian_correct_normals  = normals;
            transform_endianness(endian_correct_vertices, endian_correct_normals);
            write_binary_data(endian_correct_vertices, endian_correct_normals);
        }
        else
        {
            write_binary_data(vertices, normals);
        }
    }
}

/**
 * @brief
 * Writes the given mesh to a ply file in the ply
 * format of choice at the location given by filepath.
 * @tparam Point The mesh's point type
 * @tparam SharedVertexMeshTriangle The mesh's triangle type
 * @param filepath Path to the ply file to write to
 * @param vertices 
 * @param triangles 
 * @param format The desired ply format with which to write
 */
template <
    class Point,
    class SharedVertexMeshTriangle,
    std::enable_if_t<traits::is_shared_vertex_mesh_triangle_v<SharedVertexMeshTriangle>, int> = 0>
void write_ply(
    std::filesystem::path const& filepath,
    std::vector<Point> const& vertices,
    std::vector<SharedVertexMeshTriangle> const& triangles,
    ply_format_t format = ply_format_t::ascii)
{
    if (!filepath.has_extension() || filepath.extension() != ".ply")
        return;

    if (vertices.empty())
        return;

    if (triangles.empty())
        return;

    std::ofstream ofs{filepath.c_str(), std::ios::binary};

    if (!ofs.is_open())
        return;

    write_ply<Point, SharedVertexMeshTriangle>(ofs, vertices, triangles, format);
}

template <
    class Point,
    class SharedVertexMeshTriangle,
    std::enable_if_t<traits::is_shared_vertex_mesh_triangle_v<SharedVertexMeshTriangle>, int> = 0>
void write_ply(
    std::ostream& os,
    std::vector<Point> const& vertices,
    std::vector<SharedVertexMeshTriangle> const& triangles,
    ply_format_t format = ply_format_t::ascii)
{
    using point_type            = Point;
    using triangle_type         = SharedVertexMeshTriangle;
    using vertex_component_type = typename point_type::coordinate_type;
    // cannot use typename triangle_type::index_type because it
    // might not fit into a ply int/uint which is 4 bytes
    using index_type = std::uint32_t;

    std::string const vertex_component_type_str =
        std::is_same_v<vertex_component_type, double> ? "double" : "float";

    // we will not be using ints, because indices should always be >= 0
    std::string const index_component_type_str = "uint";

    std::ostringstream header_stream{};
    header_stream << "ply\n";

    if (format == ply_format_t::ascii)
        header_stream << "format ascii 1.0\n";
    if (format == ply_format_t::binary_little_endian)
        header_stream << "format binary_little_endian 1.0\n";
    if (format == ply_format_t::binary_big_endian)
        header_stream << "format binary_big_endian 1.0\n";

    header_stream << "element vertex " << vertices.size() << "\n"
                  << "property " << vertex_component_type_str << " x\n"
                  << "property " << vertex_component_type_str << " y\n"
                  << "property " << vertex_component_type_str << " z\n"
                  << "element face " << triangles.size() << "\n"
                  << "property list uchar " << index_component_type_str << " vertex_indices\n"
                  << "end_header\n";

    std::string const header = header_stream.str();
    os << header;

    if (format == ply_format_t::ascii)
    {
        for (point_type const& v : vertices)
        {
            std::ostringstream oss{};
            oss << std::to_string(v.x()) << " " << std::to_string(v.y()) << " "
                << std::to_string(v.z()) << "\n";
            os << oss.str();
        }
        for (triangle_type const& f : triangles)
        {
            std::ostringstream oss{};
            auto const& indices = f.indices();
            oss << "3 " << std::to_string(indices[0]) << " " << std::to_string(indices[1]) << " "
                << std::to_string(indices[2]) << "\n";
            os << oss.str();
        }
    }

    // Note: disk I/O writes can be parallelized
    auto const write_binary_data =
        [&os](std::vector<point_type> const& p, std::vector<triangle_type> const& t) {
            for (std::size_t i = 0u; i < p.size(); ++i)
            {
                auto constexpr size_of_vertex_component_type = sizeof(vertex_component_type);
                std::array<std::byte, 3u * size_of_vertex_component_type> vertex_storage;

                std::byte* const data = vertex_storage.data();

                vertex_component_type* const x = reinterpret_cast<float*>(data);
                vertex_component_type* const y =
                    reinterpret_cast<float*>(data + (1u * size_of_vertex_component_type));
                vertex_component_type* const z =
                    reinterpret_cast<float*>(data + (2u * size_of_vertex_component_type));
                *x = p[i].x();
                *y = p[i].y();
                *z = p[i].z();

                os.write(
                    reinterpret_cast<const char*>(data),
                    static_cast<std::streamsize>(vertex_storage.size()));
            }

            for (std::size_t i = 0u; i < t.size(); ++i)
            {
                auto constexpr size_of_index_type = sizeof(index_type);
                std::array<std::byte, 3u * size_of_index_type> index_storage;

                std::byte* const data = index_storage.data();

                index_type* const id1 = reinterpret_cast<index_type*>(data);
                index_type* const id2 = reinterpret_cast<index_type*>(data + (1u * size_of_index_type));
                index_type* const id3 = reinterpret_cast<index_type*>(data + (2u * size_of_index_type));
                auto const& indices   = t[i].indices();
                *id1                  = indices[0];
                *id2                  = indices[1];
                *id3                  = indices[2];

                os.write(
                    reinterpret_cast<const char*>(data),
                    static_cast<std::streamsize>(index_storage.size()));
            }
        };

    auto const transform_endianness = [](std::vector<point_type>& p,
                                         std::vector<triangle_type>& t) {
        std::transform(std::begin(p), std::end(p), std::begin(p), [](point_type const& point) {
            return point_type{
                reverse_endianness(point.x()),
                reverse_endianness(point.y()),
                reverse_endianness(point.z())};
        });
        std::transform(
            std::begin(t),
            std::end(t),
            std::begin(t),
            [](triangle_type const& triangle) {
                auto const& indices = triangle.indices();
                return triangle_type{
                    reverse_endianness(indices[0]),
                    reverse_endianness(indices[1]),
                    reverse_endianness(indices[2])};
            });
    };

    if (format == ply_format_t::binary_little_endian)
    {
        if (!is_machine_little_endian())
        {
            auto endian_correct_vertices = vertices;
            auto endian_correct_triangles  = triangles;
            transform_endianness(endian_correct_vertices, endian_correct_triangles);
            write_binary_data(endian_correct_vertices, endian_correct_triangles);
        }
        else
        {
            write_binary_data(vertices, triangles);
        }
    }

    if (format == ply_format_t::binary_big_endian)
    {
        if (!is_machine_big_endian())
        {
            auto endian_correct_vertices = vertices;
            auto endian_correct_triangles = triangles;
            transform_endianness(endian_correct_vertices, endian_correct_triangles);
            write_binary_data(endian_correct_vertices, endian_correct_triangles);
        }
        else
        {
            write_binary_data(vertices, triangles);
        }
    }
}

template <class Point, class Normal>
inline auto read_ply_ascii(std::istream& is, ply_parameters_t const& params)
    -> std::tuple<std::vector<Point>, std::vector<Normal>>
{
    using point_type  = Point;
    using normal_type = Normal;

    std::vector<point_type> vertices;
    std::vector<normal_type> normals;

    vertices.reserve(params.vertex_count);
    normals.reserve(params.normal_count);

    std::string line;

    for (std::size_t i = 0; i < params.vertex_count; ++i)
    {
        std::getline(is, line);
        if (is.bad())
            return {};

        auto const tokens = tokenize(line);
        auto const x      = std::stof(tokens.at(0));
        auto const y      = std::stof(tokens.at(1));
        auto const z      = std::stof(tokens.at(2));
        point_type const p{x, y, z};
        vertices.push_back(p);
    }
    for (std::size_t i = 0; i < params.normal_count; ++i)
    {
        std::getline(is, line);
        if (is.bad())
            return {};

        auto const tokens = tokenize(line);
        auto const nx     = std::stof(tokens.at(0));
        auto const ny     = std::stof(tokens.at(1));
        auto const nz     = std::stof(tokens.at(2));
        normal_type const n{nx, ny, nz};
        normals.push_back(n);
    }

    return std::make_tuple(vertices, normals);
}

template <class Point, class Normal>
inline auto read_ply_binary(std::istream& is, ply_parameters_t const& params)
    -> std::tuple<std::vector<Point>, std::vector<Normal>>
{
    using point_type  = Point;
    using normal_type = Normal;

    std::vector<point_type> vertices;
    std::vector<normal_type> normals;

    vertices.resize(params.vertex_count);
    normals.resize(params.normal_count);

    auto const size_of_vertex_component = sizeof(float);
    auto const size_of_normal_component = sizeof(float);

    std::vector<std::byte> vertex_component_storage;
    vertex_component_storage.resize(3u * size_of_vertex_component);

    std::vector<std::byte> normal_component_storage;
    normal_component_storage.resize(3u * size_of_normal_component);

    // Note: disk I/O reads could be parallelized, but checkout for memory consumption
    for (std::size_t i = 0; i < vertices.size(); ++i)
    {
        std::byte* data = vertex_component_storage.data();
        is.read(
            reinterpret_cast<char*>(data),
            static_cast<std::streamsize>(vertex_component_storage.size()));

        float const* const x = reinterpret_cast<float const*>(data);
        float const* const y = reinterpret_cast<float const*>(data + (1u * sizeof(float)));
        float const* const z = reinterpret_cast<float const*>(data + (2u * sizeof(float)));
        vertices[i].x(*x);
        vertices[i].y(*y);
        vertices[i].z(*z);
    }

    for (std::size_t i = 0; i < normals.size(); ++i)
    {
        std::byte* data = normal_component_storage.data();
        is.read(
            reinterpret_cast<char*>(data),
            static_cast<std::streamsize>(normal_component_storage.size()));

        float const* const nx = reinterpret_cast<float const*>(data);
        float const* const ny = reinterpret_cast<float const*>(data + (1u * sizeof(float)));
        float const* const nz = reinterpret_cast<float const*>(data + (2u * sizeof(float)));
        normals[i].nx(*nx);
        normals[i].ny(*ny);
        normals[i].nz(*nz);
    }

    if (is.bad())
        return {};

    return std::make_tuple(vertices, normals);
}

template <class Point, class Normal>
inline auto read_ply_binary_little_endian(std::istream& is, ply_parameters_t const& params)
    -> std::tuple<std::vector<Point>, std::vector<Normal>>
{
    using point_type  = Point;
    using normal_type = Normal;

    auto point_cloud = read_ply_binary<Point, Normal>(is, params);

    if (is_machine_little_endian())
        return point_cloud;

    auto& [vertices, normals] = point_cloud;

    std::transform(
        std::begin(vertices),
        std::end(vertices),
        std::begin(vertices),
        [](point_type const& p) {
            return point_type{
                reverse_endianness(p.x()),
                reverse_endianness(p.y()),
                reverse_endianness(p.z())};
        });

    std::transform(
        std::begin(normals),
        std::end(normals),
        std::begin(normals),
        [](normal_type const& n) {
            return normal_type{
                reverse_endianness(n.nx()),
                reverse_endianness(n.ny()),
                reverse_endianness(n.nz())};
        });

    return point_cloud;
}

template <class Point, class Normal>
inline auto read_ply_binary_big_endian(std::istream& is, ply_parameters_t const& params)
    -> std::tuple<std::vector<Point>, std::vector<Normal>>
{
    using point_type  = Point;
    using normal_type = Normal;

    auto point_cloud = read_ply_binary<Point, Normal>(is, params);

    if (is_machine_big_endian())
        return point_cloud;

    auto& [vertices, normals] = point_cloud;

    std::transform(
        std::begin(vertices),
        std::end(vertices),
        std::begin(vertices),
        [](point_type const& p) {
            return point_type{
                reverse_endianness(p.x()),
                reverse_endianness(p.y()),
                reverse_endianness(p.z())};
        });

    std::transform(
        std::begin(normals),
        std::end(normals),
        std::begin(normals),
        [](normal_type const& n) {
            return normal_type{
                reverse_endianness(n.nx()),
                reverse_endianness(n.ny()),
                reverse_endianness(n.nz())};
        });

    return point_cloud;
}

} // namespace io
} // namespace pcp