#ifndef PCP_IO_PLY_HPP
#define PCP_IO_PLY_HPP

/**
 * @file
 * @ingroup io
 */

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

/**
 * @ingroup io-ply
 * @brief
 * ply format types on disk
 */
enum class ply_format_t { ascii, binary_little_endian, binary_big_endian };

/**
 * @ingroup io-ply
 * @brief
 * ply point cloud point coordinate types can only be
 * float or double (single precision or double precision).
 */
enum class ply_property_type_t { single_precision, double_precision, uchar };

/**
 * @ingroup io-ply
 * @brief
 * Structure of ply parameters read dynamically from disk
 */
struct ply_parameters_t
{
    ply_format_t format                       = ply_format_t::ascii;
    std::size_t vertex_count                  = 0u;
    std::size_t normal_count                  = 0u;
    std::size_t color_count                   = 0u;
    ply_property_type_t vertex_component_type = ply_property_type_t::single_precision;
    ply_property_type_t normal_component_type = ply_property_type_t::single_precision;
    ply_property_type_t color_component_type  = ply_property_type_t::uchar;
    std::size_t color_property_offset         = 0u;
    std::size_t normal_property_offset        = 0u;
};

/**
 * @ingroup io-ply
 * @brief
 * Convert a string representation of a ply format to a ply_format_t enum value
 * @param s The string representation of a ply format
 * @return The corresponding enum value
 */
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
inline auto read_ply(std::istream& is) -> std::tuple<
    std::vector<Point>,
    std::vector<Normal>,
    std::vector<std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>>>;

template <class Point, class Normal>
inline auto read_ply_ascii(std::istream& is, ply_parameters_t const& params) -> std::tuple<
    std::vector<Point>,
    std::vector<Normal>,
    std::vector<std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>>>;

template <class Point, class Normal>
inline auto read_ply_binary(std::istream& is, ply_parameters_t const& params) -> std::tuple<
    std::vector<Point>,
    std::vector<Normal>,
    std::vector<std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>>>;

template <class Point, class Normal>
inline auto read_ply_binary_little_endian(std::istream& is, ply_parameters_t const& params)
    -> std::tuple<
        std::vector<Point>,
        std::vector<Normal>,
        std::vector<std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>>>;

template <class Point, class Normal>
inline auto read_ply_binary_big_endian(std::istream& is, ply_parameters_t const& params)
    -> std::tuple<
        std::vector<Point>,
        std::vector<Normal>,
        std::vector<std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>>>;

/**
 * @ingroup io-ply
 * @brief
 * Reads a ply file into a point cloud (potentially with normals).
 * The function supports ascii, binary little endian, binary big endian.
 * The ply file must have x,y,z properties and nx,ny,nz properties if
 * normals are available and ignores all other properties. The properties
 * must be floats.
 * @tparam Point Type of the point cloud's points to return.
 * @tparam Normal Type of the point cloud's normals to return.
 * @param path Path to the ply file to read in to memory
 * @return Returns the point cloud as a tuple of vector of points and vector of normals.
 */
template <class Point, class Normal>
inline auto read_ply(std::filesystem::path const& path) -> std::tuple<
    std::vector<Point>,
    std::vector<Normal>,
    std::vector<std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>>>
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

/**
 * @ingroup io-ply
 * @brief
 * Reads a ply file into a point cloud (potentially with normals).
 * The function supports ascii, binary little endian, binary big endian.
 * The ply file must have x,y,z properties, nx,ny,nz properties if
 * normals are available, r,g,b properties if colors are available and ignores all other properties.
 * The properties x,y,z,nx,ny,nz must be float and the properties r,g,b must be uchar.
 * @tparam Point Type of the point cloud's points to return.
 * @tparam Normal Type of the point cloud's normals to return.
 * @param is Input stream in ply format
 * @return Returns the point cloud as a tuple of vector of points, vector of normals and vector of
 * colors.
 */
template <class Point, class Normal>
inline auto read_ply(std::istream& is) -> std::tuple<
    std::vector<Point>,
    std::vector<Normal>,
    std::vector<std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>> /* colors */>
{
    static_assert(traits::is_point_v<Point>, "Point must satisfy Point concept");
    static_assert(traits::is_normal_v<Normal>, "Normal must satisfy Normal concept");

    ply_parameters_t ply_params;

    auto const string_to_property_type = [](std::string const& s) -> ply_property_type_t {
        ply_property_type_t type = ply_property_type_t::single_precision;
        if (s == "float")
            type = ply_property_type_t::single_precision;
        if (s == "double")
            type = ply_property_type_t::double_precision;
        if (s == "uchar")
            type = ply_property_type_t::uchar;
        return type;
    };

    auto const has_valid_vertex_properties = [string_to_property_type, &is](
                                                 std::array<std::string, 3> const& property_names,
                                                 ply_property_type_t& coordinate_type) -> bool {
        std::array<std::string, 3> props;
        std::array<ply_property_type_t, 3> types;
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

            types[i]        = string_to_property_type(tokens.at(1));
            coordinate_type = types[i];
            props[i]        = tokens.back();
        }

        bool const are_components_of_same_type = std::all_of(
            std::cbegin(types),
            std::cend(types),
            [coordinate_type](ply_property_type_t const& t) { return coordinate_type == t; });

        bool const are_components_well_named = props[0] == property_names[0] &&
                                               props[1] == property_names[1] &&
                                               props[2] == property_names[2];

        return are_components_of_same_type && are_components_well_named;
    };

    std::string line;
    bool is_ply                         = false;
    std::streampos previous_position    = is.tellg();
    std::size_t current_property_offset = 0u;

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

            previous_position = is.tellg();
            current_property_offset += 3u;

            continue;
        }

        if (tokens.front() == "property" && tokens.at(2) == "nx")
        {
            is.seekg(previous_position);
            ply_params.normal_count = ply_params.vertex_count;

            bool const is_normal_valid =
                has_valid_vertex_properties({"nx", "ny", "nz"}, ply_params.normal_component_type);

            if (!is_normal_valid)
                return {};

            ply_params.normal_property_offset = current_property_offset;
            current_property_offset += 3u;
            previous_position = is.tellg();

            continue;
        }

        if (tokens.front() == "property" && (tokens.at(2) == "r" || tokens.at(2) == "red"))
        {
            is.seekg(previous_position);

            ply_params.color_count = ply_params.vertex_count;

            bool is_color_valid =
                has_valid_vertex_properties({"r", "g", "b"}, ply_params.color_component_type);

            if (!is_color_valid)
            {
                is.seekg(previous_position);
                is_color_valid = has_valid_vertex_properties(
                    {"red", "green", "blue"},
                    ply_params.color_component_type);

                if (!is_color_valid)
                    return {};
            }

            ply_params.color_property_offset = current_property_offset;
            current_property_offset += 3u;
            previous_position = is.tellg();

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
 * @ingroup io-ply
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
    std::vector<std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>> const& colors,
    ply_format_t format = ply_format_t::ascii)
{
    if (!filepath.has_extension() || filepath.extension() != ".ply")
        return;

    if (vertices.empty())
        return;

    std::ofstream ofs{filepath.c_str(), std::ios::binary};

    if (!ofs.is_open())
        return;

    write_ply<Point, Normal>(ofs, vertices, normals, colors, format);
}

/**
 * @ingroup io-ply
 * @brief
 * Writes a point cloud in ply format to an output stream.
 * @tparam Point Type of points in the point cloud satisfying Point concept.
 * @tparam Normal Type of normals in the point cloud satisfying Normal concept.
 * @param os The output stream to write to
 * @param vertices The points in the point cloud
 * @param normals The normals in the point cloud
 * @param format The format in which to write the ply point cloud
 */
template <class Point, class Normal, std::enable_if_t<traits::is_normal_v<Normal>, int> = 0>
inline void write_ply(
    std::ostream& os,
    std::vector<Point> const& vertices,
    std::vector<Normal> const& normals,
    std::vector<std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>> const& colors,
    ply_format_t format = ply_format_t::ascii)
{
    using point_type            = Point;
    using normal_type           = Normal;
    using color_type            = std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>;
    using vertex_component_type = typename Point::coordinate_type;
    using normal_component_type = typename Normal::component_type;
    using color_component_type  = std::uint8_t;

    std::string const vertex_component_type_str =
        std::is_same_v<vertex_component_type, double> ? "double" : "float";

    std::string const normal_component_type_str =
        std::is_same_v<normal_component_type, double> ? "double" : "float";

    std::string const color_component_type_str = "uchar";

    bool const has_normals = normals.size() == vertices.size();
    bool const has_colors  = colors.size() == vertices.size();

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
                  << "property " << vertex_component_type_str << " z\n";

    if (has_normals)
    {
        header_stream << "property " << normal_component_type_str << " nx\n"
                      << "property " << normal_component_type_str << " ny\n"
                      << "property " << normal_component_type_str << " nz\n";
    }
    if (has_colors)
    {
        header_stream << "property " << color_component_type_str << " r\n"
                      << "property " << color_component_type_str << " g\n"
                      << "property " << color_component_type_str << " b\n";
    }

    header_stream << "end_header\n";

    std::string const header = header_stream.str();
    os << header;

    if (format == ply_format_t::ascii)
    {
        for (std::size_t i = 0u; i < vertices.size(); ++i)
        {
            auto const& v = vertices[i];
            std::ostringstream oss{};
            oss << std::to_string(v.x()) << " " << std::to_string(v.y()) << " "
                << std::to_string(v.z());

            if (has_normals)
            {
                auto const& n = normals[i];
                oss << " ";
                oss << std::to_string(n.nx()) << " " << std::to_string(n.ny()) << " "
                    << std::to_string(n.nz());
            }
            if (has_colors)
            {
                auto const& c = colors[i];
                oss << " ";
                oss << std::to_string(std::get<0>(c)) << " " << std::to_string(std::get<1>(c))
                    << " " << std::to_string(std::get<2>(c));
            }

            oss << "\n";
            os << oss.str();
        }
    }

    // Note: disk I/O writes can be parallelized
    auto const write_binary_data = [&os, &has_normals, &has_colors](
                                       std::vector<point_type> const& p,
                                       std::vector<normal_type> const& n,
                                       std::vector<color_type> const& c) {
        for (std::size_t i = 0u; i < p.size(); ++i)
        {
            auto constexpr size_of_vertex_component_type = sizeof(vertex_component_type);
            std::array<std::byte, 3u * size_of_vertex_component_type> vertex_storage;

            std::byte* const pos_data = vertex_storage.data();

            vertex_component_type* const x = reinterpret_cast<float*>(pos_data);
            vertex_component_type* const y =
                reinterpret_cast<float*>(pos_data + (1u * size_of_vertex_component_type));
            vertex_component_type* const z =
                reinterpret_cast<float*>(pos_data + (2u * size_of_vertex_component_type));
            *x = p[i].x();
            *y = p[i].y();
            *z = p[i].z();

            os.write(
                reinterpret_cast<const char*>(pos_data),
                static_cast<std::streamsize>(vertex_storage.size()));

            if (has_normals)
            {
                auto constexpr size_of_normal_component_type = sizeof(normal_component_type);
                std::array<std::byte, 3u * size_of_normal_component_type> normal_storage;

                std::byte* const normal_data = normal_storage.data();

                normal_component_type* const nx = reinterpret_cast<float*>(normal_data);
                normal_component_type* const ny =
                    reinterpret_cast<float*>(normal_data + (1u * size_of_normal_component_type));
                normal_component_type* const nz =
                    reinterpret_cast<float*>(normal_data + (2u * size_of_normal_component_type));
                *nx = n[i].nx();
                *ny = n[i].ny();
                *nz = n[i].nz();

                os.write(
                    reinterpret_cast<const char*>(normal_data),
                    static_cast<std::streamsize>(normal_storage.size()));
            }
            if (has_colors)
            {
                auto constexpr size_of_color_component_type = sizeof(color_component_type);
                std::array<std::byte, 3u * size_of_color_component_type> color_storage;

                std::byte* const color_data = color_storage.data();

                color_component_type* const r = reinterpret_cast<color_component_type*>(color_data);
                color_component_type* const g = reinterpret_cast<color_component_type*>(
                    color_data + (1u * size_of_color_component_type));
                color_component_type* const b = reinterpret_cast<color_component_type*>(
                    color_data + (2u * size_of_color_component_type));
                *r = std::get<0>(c[i]);
                *g = std::get<1>(c[i]);
                *b = std::get<2>(c[i]);

                os.write(
                    reinterpret_cast<const char*>(color_data),
                    static_cast<std::streamsize>(color_storage.size()));
            }
        }
    };

    auto const transform_endianness = [](std::vector<point_type>& p,
                                         std::vector<normal_type>& n,
                                         std::vector<color_type>& c) {
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
        std::transform(std::begin(c), std::end(c), std::begin(c), [](color_type const& color) {
            return std::make_tuple(
                reverse_endianness(std::get<0>(color)),
                reverse_endianness(std::get<1>(color)),
                reverse_endianness(std::get<2>(color)));
        });
    };

    if (format == ply_format_t::binary_little_endian)
    {
        if (!is_machine_little_endian())
        {
            auto endian_correct_vertices = vertices;
            auto endian_correct_normals  = normals;
            auto endian_correct_colors   = colors;
            transform_endianness(
                endian_correct_vertices,
                endian_correct_normals,
                endian_correct_colors);
            write_binary_data(
                endian_correct_vertices,
                endian_correct_normals,
                endian_correct_colors);
        }
        else
        {
            write_binary_data(vertices, normals, colors);
        }
    }

    if (format == ply_format_t::binary_big_endian)
    {
        if (!is_machine_big_endian())
        {
            auto endian_correct_vertices = vertices;
            auto endian_correct_normals  = normals;
            auto endian_correct_colors   = colors;
            transform_endianness(
                endian_correct_vertices,
                endian_correct_normals,
                endian_correct_colors);
            write_binary_data(
                endian_correct_vertices,
                endian_correct_normals,
                endian_correct_colors);
        }
        else
        {
            write_binary_data(vertices, normals, colors);
        }
    }
}

/**
 * @ingroup io-ply
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

/**
 * @ingroup io-ply
 * @brief
 * Writes a shared vertex triangular mesh data structure in ply format to an output stream.
 * @tparam Point Type of points in the mesh.
 * @tparam SharedVertexMeshTriangle Type of triangles in the mesh.
 * @param os The output stream to write to
 * @param vertices The points in the mesh.
 * @param triangles The triangles in the mesh.
 * @param format The ply format in which the mesh will be written
 */
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
                auto const& indices               = t[i].indices();
                std::uint8_t const num_indices    = 3;
                os.write(reinterpret_cast<const char*>(&num_indices), std::streamsize{1u});
                os.write(
                    reinterpret_cast<const char*>(indices.data()),
                    static_cast<std::streamsize>(indices.size() * size_of_index_type));
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
            auto endian_correct_vertices  = vertices;
            auto endian_correct_triangles = triangles;
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
            auto endian_correct_vertices  = vertices;
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

/**
 * @ingroup io-ply
 * @brief
 * Reads an ascii formatted ply point cloud
 * @tparam Point
 * @tparam Normal
 * @param is The input stream from which to read
 * @param params The ply parameters extracted from the ply header
 * @return
 */
template <class Point, class Normal>
inline auto read_ply_ascii(std::istream& is, ply_parameters_t const& params) -> std::tuple<
    std::vector<Point>,
    std::vector<Normal>,
    std::vector<std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>>>
{
    using point_type  = Point;
    using normal_type = Normal;
    using color_type  = std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>;

    std::vector<point_type> vertices;
    std::vector<normal_type> normals;
    std::vector<color_type> colors;

    vertices.reserve(params.vertex_count);
    normals.reserve(params.normal_count);
    colors.reserve(params.color_count);

    bool const has_normals = params.normal_count == params.vertex_count;
    bool const has_colors  = params.color_count == params.vertex_count;

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

        if (has_normals)
        {
            auto const nx = std::stof(tokens.at(params.normal_property_offset + 0u));
            auto const ny = std::stof(tokens.at(params.normal_property_offset + 1u));
            auto const nz = std::stof(tokens.at(params.normal_property_offset + 2u));
            normal_type const n{nx, ny, nz};
            normals.push_back(n);
        }
        if (has_colors)
        {
            auto const r =
                static_cast<std::uint8_t>(std::stoul(tokens.at(params.color_property_offset + 0u)));
            auto const g =
                static_cast<std::uint8_t>(std::stoul(tokens.at(params.color_property_offset + 1u)));
            auto const b =
                static_cast<std::uint8_t>(std::stoul(tokens.at(params.color_property_offset + 2u)));
            color_type const c = std::make_tuple(r, g, b);
            colors.push_back(c);
        }
    }

    return std::make_tuple(vertices, normals, colors);
}

/**
 * @ingroup io-ply
 * @brief
 * Reads a binary formatted ply point cloud
 * @tparam Point
 * @tparam Normal
 * @param is The input stream from which to read
 * @param params The ply parameters extracted from the ply header
 * @return
 */
template <class Point, class Normal>
inline auto read_ply_binary(std::istream& is, ply_parameters_t const& params) -> std::tuple<
    std::vector<Point>,
    std::vector<Normal>,
    std::vector<std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>>>
{
    using point_type  = Point;
    using normal_type = Normal;
    using color_type  = std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>;

    std::vector<point_type> vertices;
    std::vector<normal_type> normals;
    std::vector<color_type> colors;

    vertices.resize(params.vertex_count);
    normals.resize(params.normal_count);
    colors.resize(params.color_count);

    bool const has_normals = params.normal_count == params.vertex_count;
    bool const has_colors  = params.color_count == params.vertex_count;

    auto const size_of_vertex_component = sizeof(float);
    auto const size_of_normal_component = has_normals ? sizeof(float) : 0u;
    auto const size_of_color_component  = has_colors ? sizeof(std::uint8_t) : 0u;

    std::vector<std::byte> property_storage;
    property_storage.resize(
        3u * size_of_vertex_component + 3u * size_of_normal_component +
        3u * size_of_color_component);

    // Note: disk I/O reads could be parallelized, but checkout for memory consumption
    for (std::size_t i = 0; i < vertices.size(); ++i)
    {
        std::byte* data = property_storage.data();
        is.read(
            reinterpret_cast<char*>(data),
            static_cast<std::streamsize>(property_storage.size()));

        float const* const x = reinterpret_cast<float const*>(data);
        float const* const y =
            reinterpret_cast<float const*>(data + (1u * size_of_vertex_component));
        float const* const z =
            reinterpret_cast<float const*>(data + (2u * size_of_vertex_component));
        vertices[i].x(*x);
        vertices[i].y(*y);
        vertices[i].z(*z);

        if (has_normals)
        {
            auto const normal_offset =
                (has_colors && params.normal_property_offset > params.color_property_offset) ?
                    3u * size_of_vertex_component + 3u * size_of_color_component :
                    3u * size_of_vertex_component;
            float const* const nx = reinterpret_cast<float const*>(data + normal_offset);
            float const* const ny = reinterpret_cast<float const*>(
                data + normal_offset + (1u * size_of_normal_component));
            float const* const nz = reinterpret_cast<float const*>(
                data + normal_offset + (2u * size_of_normal_component));
            normals[i].nx(*nx);
            normals[i].ny(*ny);
            normals[i].nz(*nz);
        }
        if (has_colors)
        {
            auto const color_offset =
                (has_normals && params.color_property_offset > params.normal_property_offset) ?
                    3u * size_of_vertex_component + 3u * size_of_normal_component :
                    3u * size_of_vertex_component;
            std::uint8_t const* const r =
                reinterpret_cast<std::uint8_t const*>(data + color_offset);
            std::uint8_t const* const g = reinterpret_cast<std::uint8_t const*>(
                data + color_offset + (1u * size_of_color_component));
            std::uint8_t const* const b = reinterpret_cast<std::uint8_t const*>(
                data + color_offset + (2u * size_of_color_component));
            colors[i] = std::make_tuple(*r, *g, *b);
        }
    }

    if (is.bad())
        return {};

    return std::make_tuple(vertices, normals, colors);
}

/**
 * @ingroup io-ply
 * @brief
 * Reads a binary little endian formatted ply point cloud
 * @tparam Point
 * @tparam Normal
 * @param is The input stream from which to read
 * @param params The ply parameters extracted from the ply header
 * @return
 */
template <class Point, class Normal>
inline auto read_ply_binary_little_endian(std::istream& is, ply_parameters_t const& params)
    -> std::tuple<
        std::vector<Point>,
        std::vector<Normal>,
        std::vector<std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>>>
{
    using point_type  = Point;
    using normal_type = Normal;
    using color_type  = std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>;

    auto point_cloud = read_ply_binary<Point, Normal>(is, params);

    if (is_machine_little_endian())
        return point_cloud;

    auto& [vertices, normals, colors] = point_cloud;

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

    std::transform(
        std::begin(colors),
        std::end(colors),
        std::begin(colors),
        [](color_type const& c) {
            return std::make_tuple(
                reverse_endianness(std::get<0>(c)),
                reverse_endianness(std::get<1>(c)),
                reverse_endianness(std::get<2>(c)));
        });

    return point_cloud;
}

/**
 * @ingroup io-ply
 * @brief
 * Reads binary big endian formatted ply point cloud
 * @tparam Point
 * @tparam Normal
 * @param is The input stream from which to read
 * @param params The ply parameters extracted from the ply header
 * @return
 */
template <class Point, class Normal>
inline auto read_ply_binary_big_endian(std::istream& is, ply_parameters_t const& params)
    -> std::tuple<
        std::vector<Point>,
        std::vector<Normal>,
        std::vector<std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>>>
{
    using point_type  = Point;
    using normal_type = Normal;
    using color_type  = std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>;

    auto point_cloud = read_ply_binary<Point, Normal>(is, params);

    if (is_machine_big_endian())
        return point_cloud;

    auto& [vertices, normals, colors] = point_cloud;

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

    std::transform(
        std::begin(colors),
        std::end(colors),
        std::begin(colors),
        [](color_type const& c) {
            return std::make_tuple(
                reverse_endianness(std::get<0>(c)),
                reverse_endianness(std::get<1>(c)),
                reverse_endianness(std::get<2>(c)));
        });

    return point_cloud;
}

} // namespace io
} // namespace pcp

#endif // PCP_IO_PLY_HPP