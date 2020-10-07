#pragma once

#include <vector>
#include <filesystem>
#include <fstream>
#include <string>
#include <sstream>
#include <tuple>
#include <algorithm>
#include <array>

#include "point.hpp"
#include "normal.hpp"
#include "tokenize.hpp"
#include "endianness.hpp"

namespace ply {

enum class format_t
{
	ascii, binary_little_endian, binary_big_endian
};

enum class coordinate_type_t
{
	single_precision, double_precision
};

struct ply_parameters_t
{
	format_t format;
	std::size_t vertex_count = 0u;
	std::size_t normal_count = 0u;
	coordinate_type_t vertex_component_type;
	coordinate_type_t normal_component_type;
	std::size_t end_header_offset = 0u;
};

//inline auto read_ply(std::filesystem::path const& path)
//	-> std::tuple<std::vector<point_t>, std::vector<normal_t>>
//{
//	if (!path.has_extension() || path.extension() != "ply")
//		return {};
//
//	if (!std::filesystem::exists(path))
//		return {};
//
//	std::ifstream fs{ path.c_str() };
//
//	if (!fs.is_open())
//		return {};
//
//	return read_ply(static_cast<std::istream&>(fs));
//}

template <class T /* vertex coordinates' type */, class U /* normal components' type */>
inline auto read_ply_ascii(std::istream& is, ply_parameters_t const& params)
	-> std::tuple<std::vector<basic_point_t<T>>, std::vector<basic_normal_t<U>>>;

template <class T /* vertex coordinates' type */, class U /* normal components' type */>
inline auto read_ply_binary(std::istream& is, ply_parameters_t const& params)
	-> std::tuple<std::vector<basic_point_t<T>>, std::vector<basic_normal_t<U>>>;;

template <class T /* vertex coordinates' type */, class U /* normal components' type */>
inline auto read_ply_binary_little_endian(std::istream& is, ply_parameters_t const& params)
	-> std::tuple<std::vector<basic_point_t<T>>, std::vector<basic_normal_t<U>>>;;

template <class T /* vertex coordinates' type */, class U /* normal components' type */>
inline auto read_ply_binary_big_endian(std::istream& is, ply_parameters_t const& params)
	-> std::tuple<std::vector<basic_point_t<T>>, std::vector<basic_normal_t<U>>>;;

inline auto read_ply(std::istream& is)
	-> std::tuple<std::vector<point_t>, std::vector<normal_t>>
{
	using return_type = std::tuple<std::vector<point_t>, std::vector<normal_t>>;

	ply_parameters_t ply_params;

	auto const string_to_format = [](std::string const& s) -> format_t
	{
		format_t format = format_t::ascii;
		if (s == "ascii")
			format = format_t::ascii;
		if (s == "binary_little_endian")
			format = format_t::binary_little_endian;
		if (s == "binary_big_endian")
			format = format_t::binary_big_endian;
		return format;
	};

	auto const string_to_coordinate_type = [](std::string const& s) -> coordinate_type_t
	{
		coordinate_type_t type = coordinate_type_t::single_precision;
		if (s == "float")
			type = coordinate_type_t::single_precision;
		if (s == "double")
			type = coordinate_type_t::double_precision;
		return type;
	};

	auto const has_valid_vertex_properties = [string_to_coordinate_type](
		std::istream& is,
		std::array<std::string, 3> const& property_names,
		coordinate_type_t& coordinate_type) -> bool
	{
		std::array<std::string, 3> nxyz;
		std::array<coordinate_type_t, 3> types;
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

			types[i] = string_to_coordinate_type(tokens.at(1));
			coordinate_type = types[i];
			nxyz[i] = tokens.back();
		}

		bool const are_components_of_same_type =
			std::all_of(std::cbegin(types), std::cend(types),
				[coordinate_type](coordinate_type_t const& t) { return coordinate_type == t; });

		bool const are_components_well_named =
			nxyz[0] == property_names[0] &&
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

			bool const is_vertex_valid = has_valid_vertex_properties(
				is,
				{ "x", "y", "z" },
				ply_params.vertex_component_type
			);

			if (!is_vertex_valid)
				return {};

			continue;
		}

		if (tokens.front() == "element" && tokens.at(1) == "normal")
		{
			ply_params.normal_count = std::stoull(tokens.back());

			bool const is_normal_valid = has_valid_vertex_properties(
				is,
				{ "nx", "ny", "nz" },
				ply_params.normal_component_type
			);

			if (!is_normal_valid)
				return {};

			continue;
		}

		if (tokens.front() == "end_header")
			break;
	}

	// can only support float component types for both vertex and normal
	auto const parse_ply_ascii = [](std::istream& is, ply_parameters_t const& params) -> return_type
	{
		if (params.vertex_component_type == coordinate_type_t::single_precision &&
			params.normal_component_type == coordinate_type_t::single_precision)
		{
			return read_ply_ascii<float, float>(is, params);
		}
		else if (
			params.vertex_component_type == coordinate_type_t::single_precision &&
			params.normal_component_type == coordinate_type_t::double_precision)
		{
			return {};
		}
		else if (
			params.vertex_component_type == coordinate_type_t::double_precision &&
			params.normal_component_type == coordinate_type_t::single_precision)
		{
			return {};
		}
		else
		{
			return {};
		}
	};

	// can only support float component types for both vertex and normal
	auto const parse_ply_binary_little_endian = [](std::istream& is, ply_parameters_t const& params) -> return_type
	{
		if (params.vertex_component_type == coordinate_type_t::single_precision &&
			params.normal_component_type == coordinate_type_t::single_precision)
		{
			return read_ply_binary_little_endian<float, float>(is, params);
		}
		else if (
			params.vertex_component_type == coordinate_type_t::single_precision &&
			params.normal_component_type == coordinate_type_t::double_precision)
		{
			return {};
		}
		else if (
			params.vertex_component_type == coordinate_type_t::double_precision &&
			params.normal_component_type == coordinate_type_t::single_precision)
		{
			return {};
		}
		else
		{
			return {};
		}
	};

	// can only support float component types for both vertex and normal
	auto const parse_ply_binary_big_endian = [](std::istream& is, ply_parameters_t const& params) -> return_type
	{
		if (params.vertex_component_type == coordinate_type_t::single_precision &&
			params.normal_component_type == coordinate_type_t::single_precision)
		{
			return read_ply_binary_big_endian<float, float>(is, params);
		}
		else if (
			params.vertex_component_type == coordinate_type_t::single_precision &&
			params.normal_component_type == coordinate_type_t::double_precision)
		{
			return {};
		}
		else if (
			params.vertex_component_type == coordinate_type_t::double_precision &&
			params.normal_component_type == coordinate_type_t::single_precision)
		{
			return {};
		}
		else
		{
			return {};
		}
	};

	switch (ply_params.format)
	{
	case format_t::ascii:
		return parse_ply_ascii(is, ply_params);
	case format_t::binary_little_endian:
		return parse_ply_binary_little_endian(is, ply_params);
	case format_t::binary_big_endian:
		return parse_ply_binary_big_endian(is, ply_params);
	default:
		return {};
	}
}

/**
* Only supports T = float, T = double, U = float, U = double
*/
template <class T /* vertex component type */, class U /* normal component type */>
inline void write_ply(
	std::ostream& os, 
	std::vector<basic_point_t<T>> const& vertices, 
	std::vector<basic_normal_t<U>> const& normals, 
	format_t format = format_t::ascii)
{
	using point_type = basic_point_t<T>;
	using normal_type = basic_normal_t<U>;

	std::string const vertex_component_type =
		std::is_same_v<T, double> ? "double" : "float";

	std::string const normal_component_type =
		std::is_same_v<T, double> ? "double" : "float";

	std::ostringstream header_stream{};
	header_stream << "ply\n";

	if (format == format_t::ascii)
		header_stream << "format ascii 1.0\n";
	if (format == format_t::binary_little_endian)
		header_stream << "format binary_little_endian 1.0\n";
	if (format == format_t::binary_big_endian)
		header_stream << "format binary_big_endian 1.0\n";

	header_stream
		<< "element vertex " << vertices.size() << "\n"
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

	if (format == format_t::ascii)
	{
		for (point_type const& v : vertices)
		{
			std::ostringstream oss{};
			oss << std::to_string(x) << " " << std::to_string(y) << " " << std::to_string(z) << "\n";
			os << oss.str();
		}
		for (normal_type const& n : normals)
		{
			std::ostringstream oss{};
			oss << std::to_string(nx) << " " << std::to_string(ny) << " " << std::to_string(nz) << "\n";
			os << oss.str();
		}
	}

	auto const write_binary_data = [](
		std::ostream& os, 
		std::vector<point_type> const& vertices, 
		std::vector<normal_type> const& normals)
	{
		os.write(reinterpret_cast<const char*>(vertices.data()), vertices.size() * sizeof(point_type));
		os.write(reinterpret_cast<const char*>(normals.data()), normals.size() * sizeof(normal_type));
	};

	auto const transform_endianness = [](
		std::vector<point_type>& vertices,
		std::vector<normal_type>& normals)
	{
		std::transform(std::begin(vertices), std::end(vertices), std::begin(vertices), [](point_type const& p)
		{
			return point_type{ reverse_endianness(p.x), reverse_endianness(p.y), reverse_endianness(p.z) };
		});
		std::transform(std::begin(normals), std::end(normals), std::begin(normals), [](normal_type const& n)
		{
			return normal_type{ reverse_endianness(n.x), reverse_endianness(n.y), reverse_endianness(n.z) };
		});
	};

	if (format == format_t::binary_little_endian)
	{
		if (!is_machine_little_endian())
		{
			auto endian_correct_vertices = vertices;
			auto endian_correct_normals  = normals;
			transform_endianness(endian_correct_vertices, endian_correct_normals);
			write_binary_data(os, endian_correct_vertices, endian_correct_normals);
		}
		else
		{
			write_binary_data(os, vertices, normals);
		}
	}

	if (format == format_t::binary_big_endian)
	{
		if (!is_machine_big_endian())
		{
			auto endian_correct_vertices = vertices;
			auto endian_correct_normals = normals;
			transform_endianness(endian_correct_vertices, endian_correct_normals);
			write_binary_data(os, endian_correct_vertices, endian_correct_normals);
		}
		else
		{
			write_binary_data(os, vertices, normals);
		}
	}
}

template <class T /* vertex coordinates' type */, class U /* normal components' type */>
inline auto read_ply_ascii(std::istream& is, ply_parameters_t const& params)
	-> std::tuple<std::vector<basic_point_t<T>>, std::vector<basic_normal_t<U>>>
{
	using point_type = basic_point_t<T>;
	using normal_type = basic_normal_t<U>;

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
		auto const x = std::stof(tokens.at(0));
		auto const y = std::stof(tokens.at(1));
		auto const z = std::stof(tokens.at(2));
		point_type const p{ x, y, z };
		vertices.push_back(p);
	}
	for (std::size_t i = 0; i < params.normal_count; ++i)
	{
		std::getline(is, line);
		if (is.bad())
			return {};

		auto const tokens = tokenize(line);
		auto const nx = std::stof(tokens.at(0));
		auto const ny = std::stof(tokens.at(1));
		auto const nz = std::stof(tokens.at(2));
		normal_type const n{ nx, ny, nz };
		normals.push_back(n);
	}

	return std::make_tuple(vertices, normals);
}

template <class T /* vertex coordinates' type */, class U /* normal components' type */>
inline auto read_ply_binary(std::istream& is, ply_parameters_t const& params)
	-> std::tuple<std::vector<basic_point_t<T>>, std::vector<basic_normal_t<U>>>
{
	using point_type = basic_point_t<T>;
	using normal_type = basic_normal_t<U>;

	std::vector<point_type> vertices;
	std::vector<normal_type> normals;

	vertices.resize(params.vertex_count);
	normals.resize(params.normal_count);

	is.read(
		reinterpret_cast<char*>(vertices.data()),
		params.vertex_count * sizeof(point_type)
	);

	is.read(
		reinterpret_cast<char*>(normals.data()),
		params.normal_count * sizeof(normal_type)
	);

	if (is.bad())
		return {};

	return std::make_tuple(vertices, normals);
}

template <class T /* vertex coordinates' type */, class U /* normal components' type */>
inline auto read_ply_binary_little_endian(std::istream& is, ply_parameters_t const& params)
	-> std::tuple<std::vector<basic_point_t<T>>, std::vector<basic_normal_t<U>>>
{
	using point_type = basic_point_t<T>;
	using normal_type = basic_normal_t<U>;

	auto point_cloud = read_ply_binary<T, U>(is, params);

	if (is_machine_little_endian())
		return point_cloud;

	auto& [vertices, normals] = point_cloud;

	std::transform(std::begin(vertices), std::end(vertices), std::begin(vertices), [](point_type const& p)
	{
		return point_type{ reverse_endianness(p.x), reverse_endianness(p.y), reverse_endianness(p.z) };
	});

	std::transform(std::begin(normals), std::end(normals), std::begin(normals), [](normal_type const& p)
	{
		return normal_type{ reverse_endianness(p.x), reverse_endianness(p.y), reverse_endianness(p.z) };
	});

	return point_cloud;
}

template <class T /* vertex coordinates' type */, class U /* normal components' type */>
inline auto read_ply_binary_big_endian(std::istream& is, ply_parameters_t const& params)
	-> std::tuple<std::vector<basic_point_t<T>>, std::vector<basic_normal_t<U>>>
{
	using point_type = basic_point_t<T>;
	using normal_type = basic_normal_t<U>;

	auto point_cloud = read_ply_binary<T, U>(is, params);

	if (is_machine_big_endian())
		return point_cloud;

	auto& [vertices, normals] = point_cloud;

	std::transform(std::begin(vertices), std::end(vertices), std::begin(vertices), [](point_type const& p)
	{
		return point_type{ reverse_endianness(p.x), reverse_endianness(p.y), reverse_endianness(p.z) };
	});

	std::transform(std::begin(normals), std::end(normals), std::begin(normals), [](normal_type const& p)
	{
		return normal_type{ reverse_endianness(p.x), reverse_endianness(p.y), reverse_endianness(p.z) };
	});

	return point_cloud;
}

} // ply