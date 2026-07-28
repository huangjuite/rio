// ROS-free stand-in for <ros/console.h>.
//
// Part of the compatibility shim that lets the upstream ekf_rio / reve
// sources compile without any ROS installation. Only the logging macros
// actually reached by the compiled translation units are required
// (ROS_INFO_STREAM, ROS_WARN_STREAM, ROS_ERROR_STREAM and
// ROS_INFO_STREAM_THROTTLE); the rest are provided so that pulling in
// further upstream files does not immediately break the build.

#pragma once

#include <chrono>
#include <cstdio>
#include <iostream>
#include <sstream>

namespace rio_shim
{
enum class Level
{
  Debug = 0,
  Info  = 1,
  Warn  = 2,
  Error = 3,
  Fatal = 4,
  None  = 5
};

/// Process-wide log level. Inline so every translation unit shares one copy.
inline Level& logLevel()
{
  static Level level = Level::Warn;
  return level;
}

/// Monotonic seconds since first call, used by the *_THROTTLE macros.
inline double steadySeconds()
{
  using clock = std::chrono::steady_clock;
  static const clock::time_point t0 = clock::now();
  return std::chrono::duration<double>(clock::now() - t0).count();
}
}  // namespace rio_shim

#define RIO_SHIM_LOG(LEVEL, TAG, ARGS)                                                   \
  do                                                                                     \
  {                                                                                      \
    if (::rio_shim::logLevel() <= ::rio_shim::Level::LEVEL)                               \
    {                                                                                    \
      std::ostringstream _rio_shim_ss;                                                   \
      _rio_shim_ss << ARGS;                                                              \
      std::cerr << "[" TAG "] " << _rio_shim_ss.str() << std::endl;                      \
    }                                                                                    \
  } while (0)

// Each call site gets its own `last` because the macro expands inside the
// enclosing function body at a unique location.
#define RIO_SHIM_LOG_THROTTLE(PERIOD, LEVEL, TAG, ARGS)                                  \
  do                                                                                     \
  {                                                                                      \
    static double _rio_shim_last = -1.0e18;                                              \
    const double _rio_shim_now   = ::rio_shim::steadySeconds();                          \
    if (_rio_shim_now - _rio_shim_last >= (PERIOD))                                      \
    {                                                                                    \
      _rio_shim_last = _rio_shim_now;                                                    \
      RIO_SHIM_LOG(LEVEL, TAG, ARGS);                                                    \
    }                                                                                    \
  } while (0)

#define RIO_SHIM_LOG_ONCE(LEVEL, TAG, ARGS)                                              \
  do                                                                                     \
  {                                                                                      \
    static bool _rio_shim_done = false;                                                  \
    if (!_rio_shim_done)                                                                 \
    {                                                                                    \
      _rio_shim_done = true;                                                             \
      RIO_SHIM_LOG(LEVEL, TAG, ARGS);                                                    \
    }                                                                                    \
  } while (0)

#define ROS_DEBUG_STREAM(args) RIO_SHIM_LOG(Debug, "DEBUG", args)
#define ROS_INFO_STREAM(args) RIO_SHIM_LOG(Info, "INFO", args)
#define ROS_WARN_STREAM(args) RIO_SHIM_LOG(Warn, "WARN", args)
#define ROS_ERROR_STREAM(args) RIO_SHIM_LOG(Error, "ERROR", args)
#define ROS_FATAL_STREAM(args) RIO_SHIM_LOG(Fatal, "FATAL", args)

#define ROS_DEBUG_STREAM_THROTTLE(period, args) RIO_SHIM_LOG_THROTTLE(period, Debug, "DEBUG", args)
#define ROS_INFO_STREAM_THROTTLE(period, args) RIO_SHIM_LOG_THROTTLE(period, Info, "INFO", args)
#define ROS_WARN_STREAM_THROTTLE(period, args) RIO_SHIM_LOG_THROTTLE(period, Warn, "WARN", args)
#define ROS_ERROR_STREAM_THROTTLE(period, args) RIO_SHIM_LOG_THROTTLE(period, Error, "ERROR", args)

#define ROS_DEBUG_STREAM_ONCE(args) RIO_SHIM_LOG_ONCE(Debug, "DEBUG", args)
#define ROS_INFO_STREAM_ONCE(args) RIO_SHIM_LOG_ONCE(Info, "INFO", args)
#define ROS_WARN_STREAM_ONCE(args) RIO_SHIM_LOG_ONCE(Warn, "WARN", args)
#define ROS_ERROR_STREAM_ONCE(args) RIO_SHIM_LOG_ONCE(Error, "ERROR", args)

#define RIO_SHIM_LOG_PRINTF(LEVEL, TAG, ...)                                             \
  do                                                                                     \
  {                                                                                      \
    if (::rio_shim::logLevel() <= ::rio_shim::Level::LEVEL)                               \
    {                                                                                    \
      std::fprintf(stderr, "[" TAG "] ");                                                \
      std::fprintf(stderr, __VA_ARGS__);                                                 \
      std::fprintf(stderr, "\n");                                                        \
    }                                                                                    \
  } while (0)

#define ROS_DEBUG(...) RIO_SHIM_LOG_PRINTF(Debug, "DEBUG", __VA_ARGS__)
#define ROS_INFO(...) RIO_SHIM_LOG_PRINTF(Info, "INFO", __VA_ARGS__)
#define ROS_WARN(...) RIO_SHIM_LOG_PRINTF(Warn, "WARN", __VA_ARGS__)
#define ROS_ERROR(...) RIO_SHIM_LOG_PRINTF(Error, "ERROR", __VA_ARGS__)
#define ROS_FATAL(...) RIO_SHIM_LOG_PRINTF(Fatal, "FATAL", __VA_ARGS__)
