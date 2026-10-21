/*******************************************************
 * ROS logging/assert macros for builds with and without ROS.
 *
 * With VINS_USE_ROS (the catkin build) this is just <ros/ros.h>. Without it,
 * the ROS_* macros the estimator uses are defined here: DEBUG is compiled out,
 * INFO/WARN/ERROR go to stderr (INFO only when the VINS_LOG_INFO environment
 * variable is set - the estimator logs every optimisation step at INFO), and
 * the asserts follow <cassert> (NDEBUG disables them, as ROS_ASSERT does).
 *
 * Licensed under the GNU General Public License v3.0, as the rest of VINS.
 *******************************************************/

#pragma once

#ifdef VINS_USE_ROS
#include <ros/assert.h>
#include <ros/console.h>
#include <ros/ros.h>
#else
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <iostream>

namespace vins_log
{
inline bool infoEnabled()
{
    static const bool enabled = std::getenv("VINS_LOG_INFO") != nullptr;
    return enabled;
}
} // namespace vins_log

#define VINS_LOG_PRINTF_(level, ...)                                                                                   \
    do                                                                                                                 \
    {                                                                                                                  \
        std::fprintf(stderr, "[" level "] " __VA_ARGS__);                                                              \
        std::fputc('\n', stderr);                                                                                      \
    } while (0)
#define VINS_LOG_STREAM_(level, args)                                                                                  \
    do                                                                                                                 \
    {                                                                                                                  \
        std::cerr << "[" level "] " << args << std::endl;                                                              \
    } while (0)

#define ROS_DEBUG(...) ((void)0)
#define ROS_DEBUG_STREAM(args) ((void)0)
#define ROS_INFO(...)                                                                                                  \
    do                                                                                                                 \
    {                                                                                                                  \
        if (vins_log::infoEnabled())                                                                                   \
            VINS_LOG_PRINTF_("INFO", __VA_ARGS__);                                                                     \
    } while (0)
#define ROS_INFO_STREAM(args)                                                                                          \
    do                                                                                                                 \
    {                                                                                                                  \
        if (vins_log::infoEnabled())                                                                                   \
            VINS_LOG_STREAM_("INFO", args);                                                                            \
    } while (0)
#define ROS_WARN(...) VINS_LOG_PRINTF_("WARN", __VA_ARGS__)
#define ROS_WARN_STREAM(args) VINS_LOG_STREAM_("WARN", args)
#define ROS_ERROR(...) VINS_LOG_PRINTF_("ERROR", __VA_ARGS__)
#define ROS_ERROR_STREAM(args) VINS_LOG_STREAM_("ERROR", args)
#define ROS_ASSERT(cond) assert(cond)
#define ROS_ASSERT_MSG(cond, ...) assert(cond)
#define ROS_BREAK() std::abort()
#endif
