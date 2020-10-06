#pragma once

#include <vector>
#include <filesystem>
#include <fstream>
#include <string>
#include <sstream>
#include <tuple>

#include "point.hpp"
#include "normal.hpp"

inline auto read_obj(std::filesystem::path const& path) 
	-> std::tuple<std::vector<point_t>, std::vector<normal_t>>
{
	if (!path.has_extension() || path.extension() != "obj")
		return {};

	if (!std::filesystem::exists(path))
		return {};

	std::ifstream fs{ path.c_str() };

	if (!fs.is_open())
		return {};

	std::vector<point_t> points;
	std::vector<normal_t> normals;

	std::string line;
	while (std::getline(fs, line))
	{
		std::string type = line.substr(0, 2);

		if (type == "v ")
		{
			std::istringstream s(line.substr(2));
			point_t v;
			s >> v.x; s >> v.y; s >> v.z;
			points.push_back(v);
		}
		else if (type == "vn")
		{
			std::istringstream s(line.substr(2));
			normal_t vn;
			s >> vn.x; s >> vn.y; s >> vn.z;
			normals.push_back(vn);
		}
	}

	return std::make_tuple(points, normals);
}

inline void write_obj(std::vector<point_t> const& points, std::vector<normal_t> normals, std::filesystem::path const& path)
{
	if (!path.has_extension() || path.extension() != "obj")
		return;
	
	if (points.empty())
		return;

	if (points.size() != normals.size())
		return;

	std::ofstream fs{ path.c_str() };

	if (!fs.is_open())
		return;

	bool const has_normals = !normals.empty();
	for (std::size_t i = 0; i < points.size(); ++i)
	{
		auto const& p = points[i];
		std::string const v = "v " + 
			std::to_string(p.x) + " " + 
			std::to_string(p.y) + " " + 
			std::to_string(p.z) + "\n";

		fs << v;

		if (!has_normals)
			continue;

		auto const& n = normals[i];
		std::string const vn = "vn " +
			std::to_string(n.x) + " " +
			std::to_string(n.y) + " " +
			std::to_string(n.z) + "\n";

		fs << vn;
	}
}