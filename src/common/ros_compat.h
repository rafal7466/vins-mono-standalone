// ros_compat.h -- minimal replacement of the ROS facilities used by VINS-Mono (console macros, ROS_ASSERT and
// std_msgs::Header), so that the original estimator / feature-tracker sources compile without ROS.
#pragma once
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <iostream>
#include <sstream>
#include <string>

namespace vins_log
{
// 0 = DEBUG, 1 = INFO, 2 = WARN, 3 = ERROR, 4 = silent (set from the yaml key "log_level")
extern int level;
// optional log file (all messages >= level are also written here)
extern FILE *file;
void vprint(int lvl, const char *fmt, ...);
void sprint(int lvl, const std::string &msg);
} // namespace vins_log

#define ROS_DEBUG(...) ::vins_log::vprint(0, __VA_ARGS__)
#define ROS_INFO(...) ::vins_log::vprint(1, __VA_ARGS__)
#define ROS_WARN(...) ::vins_log::vprint(2, __VA_ARGS__)
#define ROS_ERROR(...) ::vins_log::vprint(3, __VA_ARGS__)
#define VINS_STREAM_LOG_(lvl, x)                    \
    do                                              \
    {                                               \
        if ((lvl) >= ::vins_log::level)             \
        {                                           \
            std::ostringstream vins_oss_;           \
            vins_oss_ << x;                         \
            ::vins_log::sprint((lvl), vins_oss_.str()); \
        }                                           \
    } while (0)
#define ROS_DEBUG_STREAM(x) VINS_STREAM_LOG_(0, x)
#define ROS_INFO_STREAM(x) VINS_STREAM_LOG_(1, x)
#define ROS_WARN_STREAM(x) VINS_STREAM_LOG_(2, x)
#define ROS_ERROR_STREAM(x) VINS_STREAM_LOG_(3, x)
#define ROS_ASSERT(cond)                                                                       \
    do                                                                                         \
    {                                                                                          \
        if (!(cond))                                                                           \
        {                                                                                      \
            std::fprintf(stderr, "ASSERTION FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            std::abort();                                                                      \
        }                                                                                      \
    } while (0)
#define ROS_BREAK()                                                          \
    do                                                                       \
    {                                                                        \
        std::fprintf(stderr, "ROS_BREAK at %s:%d\n", __FILE__, __LINE__); \
        std::abort();                                                        \
    } while (0)

// std_msgs::Header replacement: VINS only uses header.stamp.toSec()
namespace std_msgs
{
struct Time
{
    double sec = 0.0;
    Time() = default;
    explicit Time(double s) : sec(s) {}
    double toSec() const { return sec; }
};
struct Header
{
    unsigned int seq = 0;
    Time stamp;
    std::string frame_id;
};
} // namespace std_msgs
