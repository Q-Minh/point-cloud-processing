#ifndef PCP_COMMON_TIMER_HPP
#define PCP_COMMON_TIMER_HPP

/**
 * @file
 * @ingroup common
 */

#include <chrono>
#include <string>
#include <utility>
#include <vector>

namespace pcp {
namespace common {

/**
 * @ingroup common
 * @brief
 * Simple timer type implemented with the std::chrono::high_resolution_clock API.
 */
struct basic_timer_t
{
    using time_type     = std::chrono::high_resolution_clock::time_point;
    using duration_type = std::chrono::high_resolution_clock::duration;

    void start() { begin = std::chrono::high_resolution_clock::now(); }
    void stop()
    {
        end               = std::chrono::high_resolution_clock::now();
        ops.back().second = (end - begin);
    }
    void register_op(std::string const& op_name) { ops.push_back({op_name, {}}); }

    time_type begin;
    time_type end;
    std::vector<std::pair<std::string, duration_type>> ops;
};

} // namespace common
} // namespace pcp

#endif // PCP_COMMON_TIMER_HPP