#ifndef PCP_IO_TOKENIZE_HPP
#define PCP_IO_TOKENIZE_HPP

/**
 * @file
 * @ingroup io
 */

#include <iterator>
#include <sstream>
#include <string>
#include <vector>

namespace pcp {

/**
 * @ingroup io
 * @brief Tokenize a string by whitespace
 * @param s String to tokenize
 * @return Vector of tokens
 */
inline auto tokenize(std::string const& s) -> std::vector<std::string>
{
    std::istringstream iss(s);
    std::vector<std::string> tokens{std::istream_iterator<std::string>(iss), {}};
    return tokens;
}

} // namespace pcp

#endif // PCP_IO_TOKENIZE_HPP