#pragma once

#include <iterator>
#include <sstream>
#include <string>
#include <vector>

namespace pcp {

inline auto tokenize(std::string const& s) -> std::vector<std::string>
{
    std::istringstream iss(s);
    std::vector<std::string> tokens{std::istream_iterator<std::string>(iss), {}};
    return tokens;
}

} // namespace pcp