#pragma once

#include "endianness.hpp"
#include "normal_traits.hpp"
#include "point_traits.hpp"
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
    std::size_t end_header_offset               = 0u;
};

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

template <class Point, class Normal>
inline auto read_ply(std::filesystem::path const& path)
    -> std::tuple<std::vector<Point>, std::vector<Normal>>
{
    if (!path.has_filename())
        return {};

    if (!path.has_extension() || path.extension() != "ply")
        return {};

    if (!std::filesystem::exists(path))
        return {};

    std::ifstream fs{path.filename()};

    if (!fs.is_open())
        return {};

    return read_ply<Point, Normal>(fs);
}

template <class Point, class Normal>
inline auto read_ply(std::istream& is) -> std::tuple<std::vector<Point>, std::vector<Normal>>
{
    static_assert(traits::is_point_v<Point>, "Point must satisfy Point concept");
    static_assert(traits::is_normal_v<Normal>, "Normal must satisfy Normal concept");
    using return_type = std::tuple<std::vector<Point>, std::vector<Normal>>;

    ply_parameters_t ply_params;

    auto const string_to_format = [](std::string const& s) -> ply_format_t {
        ply_format_t format = ply_format_t::ascii;
        if (s == "ascii")
            format = ply_format_t::ascii;
        if (s == "binary_little_endian")
            format = ply_format_t::binary_little_endian;
        if (s == "binary_big_endian")
            format = ply_format_t::binary_big_endian;
        return format;
    };

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

    /**
     * Read ply header
     */
    while (std::getline(is, line))
    {
        ply_params.end_header_offset += line.size();
        auto const tokens = tokenize(line);

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

    // can only support float component types for both vertex and normal
    auto const parse_ply_ascii = [&is](ply_parameters_t const& params) -> return_type {
        return read_ply_ascii<Point, Normal>(is, params);
    };

    // can only support float component types for both vertex and normal
    auto const parse_ply_binary_little_endian =
        [&is](ply_parameters_t const& params) -> return_type {
        return read_ply_binary_little_endian<Point, Normal>(is, params);
    };

    // can only support float component types for both vertex and normal
    auto const parse_ply_binary_big_endian = [&is](ply_parameters_t const& params) -> return_type {
        return read_ply_binary_big_endian<Point, Normal>(is, params);
    };

    switch (ply_params.format)
    {
        case ply_format_t::ascii: return parse_ply_ascii(ply_params);
        case ply_format_t::binary_little_endian: return parse_ply_binary_little_endian(ply_params);
        case ply_format_t::binary_big_endian: return parse_ply_binary_big_endian(ply_params);
        default: return {};
    }
}

template <class Point, class Normal>
inline void write_ply(
    std::ostream& os,
    std::vector<Point> const& vertices,
    std::vector<Normal> const& normals,
    ply_format_t format = ply_format_t::ascii)
{
    using point_type  = Point;
    using normal_type = Normal;

    std::string const vertex_component_type =
        std::is_same_v<typename Point::coordinate_type, double> ? "double" : "float";

    std::string const normal_component_type =
        std::is_same_v<typename Normal::coordinate_type, double> ? "double" : "float";

    std::ostringstream header_stream{};
    header_stream << "ply\n";

    if (format == ply_format_t::ascii)
        header_stream << "format ascii 1.0\n";
    if (format == ply_format_t::binary_little_endian)
        header_stream << "format binary_little_endian 1.0\n";
    if (format == ply_format_t::binary_big_endian)
        header_stream << "format binary_big_endian 1.0\n";

    header_stream << "element vertex " << vertices.size() << "\n"
                  << "property " << vertex_component_type << " x\n"
                  << "property " << vertex_component_type << " y\n"
                  << "property " << vertex_component_type << " z\n"
                  << "element normal " << normals.size() << "\n"
                  << "property " << normal_component_type << " nx\n"
                  << "property " << normal_component_type << " ny\n"
                  << "property " << normal_component_type << " nz\n"
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
            oss << std::to_string(n.x()) << " " << std::to_string(n.y()) << " "
                << std::to_string(n.z()) << "\n";
            os << oss.str();
        }
    }

    auto const write_binary_data =
        [&os](std::vector<point_type> const& p, std::vector<normal_type> const& n) {
            os.write(
                reinterpret_cast<const char*>(p.data()),
                static_cast<std::streamsize>(p.size() * sizeof(point_type)));
            os.write(
                reinterpret_cast<const char*>(n.data()),
                static_cast<std::streamsize>(n.size() * sizeof(normal_type)));
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
                reverse_endianness(normal.x()),
                reverse_endianness(normal.y()),
                reverse_endianness(normal.z())};
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

    is.read(
        reinterpret_cast<char*>(vertices.data()),
        static_cast<std::streamsize>(params.vertex_count * sizeof(point_type)));

    is.read(
        reinterpret_cast<char*>(normals.data()),
        static_cast<std::streamsize>(params.normal_count * sizeof(normal_type)));

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
                reverse_endianness(n.x()),
                reverse_endianness(n.y()),
                reverse_endianness(n.z())};
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
                reverse_endianness(n.x()),
                reverse_endianness(n.y()),
                reverse_endianness(n.z())};
        });

    return point_cloud;
}

} // namespace io
} // namespace pcp