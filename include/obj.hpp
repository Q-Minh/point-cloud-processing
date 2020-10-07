#pragma once

#include <vector>
#include <filesystem>
#include <fstream>
#include <string>
#include <sstream>
#include <tuple>

#include "point.hpp"
#include "normal.hpp"

namespace pcp {
namespace io {

template <class T /* vertex component type */, class U /* normal component type */>
inline auto read_obj(std::filesystem::path const& path) 
	-> std::tuple<std::vector<basic_point_t<T>>, std::vector<basic_normal_t<U>>>
{
	if (!path.has_extension() || path.extension() != "obj")
		return {};

	if (!std::filesystem::exists(path))
		return {};

	std::ifstream ifs{ path.c_str() };

	if (!ifs.is_open())
		return {};

	return read_obj(ifs);
}

template <class T, class U>
inline auto read_obj(std::istream& is)
	-> std::tuple<std::vector<basic_point_t<T>>, std::vector<basic_normal_t<U>>>
{
	using point_type = basic_point_t<T>;
	using normal_type = basic_normal_t<U>;

	std::vector<point_type> points;
	std::vector<normal_type> normals;

	std::string line;
	while (std::getline(is, line))
	{
		std::string type = line.substr(0, 2);

		if (type == "v ")
		{
			std::istringstream s(line.substr(2));
			point_type v;
			s >> v.x; s >> v.y; s >> v.z;
			points.push_back(v);
		}
		else if (type == "vn")
		{
			std::istringstream s(line.substr(2));
			normal_type vn;
			s >> vn.x; s >> vn.y; s >> vn.z;
			normals.push_back(vn);
		}
	}

	return std::make_tuple(points, normals);
}

template <class T, class U>
inline void write_obj(std::vector<basic_point_t<T>> const& points, std::vector<basic_normal_t<U>> const& normals, std::filesystem::path const& path)
{
	if (!path.has_extension() || path.extension() != "obj")
		return;
	
	if (points.empty())
		return;

	if (points.size() != normals.size())
		return;

	std::ofstream ofs{ path.c_str() };

	if (!ofs.is_open())
		return;

	write_obj(points, normals, ofs);
}

template <class T, class U>
inline void write_obj(std::vector<basic_point_t<T>> const& points, std::vector<basic_normal_t<U>> const& normals, std::ostream& os)
{
	bool const has_normals = !normals.empty();
	for (std::size_t i = 0; i < points.size(); ++i)
	{
		auto const& p = points[i];
		std::string const v = "v " +
			std::to_string(p.x) + " " +
			std::to_string(p.y) + " " +
			std::to_string(p.z) + "\n";

		os << v;

		if (!has_normals)
			continue;

		auto const& n = normals[i];
		std::string const vn = "vn " +
			std::to_string(n.x) + " " +
			std::to_string(n.y) + " " +
			std::to_string(n.z) + "\n";

		os << vn;
	}
}

} // io
} // pcp