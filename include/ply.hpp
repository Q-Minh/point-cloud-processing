#pragma once

#include <vector>
#include <filesystem>
#include <fstream>
#include <string>
#include <sstream>
#include <tuple>
#include <algorithm>

#include "point.hpp"
#include "normal.hpp"

// TODO: finish ply parsing
inline auto read_ply(std::filesystem::path const& path)
-> std::tuple<std::vector<point_t>, std::vector<normal_t>>
{
	if (!path.has_extension() || path.extension() != "ply")
		return {};

	if (!std::filesystem::exists(path))
		return {};

	std::ifstream fs{ path.c_str() };

	if (!fs.is_open())
		return {};

	std::vector<point_t> points;
	std::vector<normal_t> normals;

	std::string line;
	bool first_line_read = false;
	bool header_read     = false;

	while (std::getline(fs, line))
	{
		if (!first_line_read)
		{
			if (line != "ply")
				return {};

			first_line_read = true;
			continue;
		}

		if (!header_read)
		{
			if (line.rfind("end_header", 0) == 0)
			{
				header_read = true;
			}
			if (line.rfind("element vertex", 0) == 0)
			{
				std::istringstream iss(line);
				std::vector<std::string> tokens{
					std::istream_iterator<std::string>(iss), {}
				};
				std::string const& ssize = tokens.back();
				std::size_t const num_points = std::stoull(ssize);
				points.reserve(num_points);
			}

			continue;
		}
	}

	return std::make_tuple(points, normals);
}