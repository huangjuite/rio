// ROS-free stand-in for <ros/time.h>.
//
// `ros::Time` here is a plain double of seconds since the epoch, which is
// exactly how the upstream filter uses it: differences, comparisons and
// `toSec()`. Keeping the `double` constructor explicit stops a stray
// `Time == double` from compiling silently.

#pragma once

#include <cstdint>
#include <iomanip>
#include <ostream>

#include <ros/console.h>

namespace ros
{
struct Duration
{
  double t_ = 0.0;

  Duration() = default;
  explicit Duration(double t) : t_{t} {}
  Duration(int32_t sec, int32_t nsec) : t_{static_cast<double>(sec) + 1.0e-9 * nsec} {}

  double toSec() const { return t_; }
  int64_t toNSec() const { return static_cast<int64_t>(t_ * 1.0e9); }

  Duration operator+(const Duration& o) const { return Duration(t_ + o.t_); }
  Duration operator-(const Duration& o) const { return Duration(t_ - o.t_); }
  Duration operator*(double s) const { return Duration(t_ * s); }

  bool operator==(const Duration& o) const { return t_ == o.t_; }
  bool operator!=(const Duration& o) const { return t_ != o.t_; }
  bool operator<(const Duration& o) const { return t_ < o.t_; }
  bool operator<=(const Duration& o) const { return t_ <= o.t_; }
  bool operator>(const Duration& o) const { return t_ > o.t_; }
  bool operator>=(const Duration& o) const { return t_ >= o.t_; }

  /// No-op: nothing in the compiled set actually needs to sleep.
  bool sleep() const { return true; }
};

struct Time
{
  double t_ = 0.0;

  Time() = default;
  explicit Time(double t) : t_{t} {}
  Time(uint32_t sec, uint32_t nsec) : t_{static_cast<double>(sec) + 1.0e-9 * nsec} {}

  static Time now() { return Time(0.0); }

  double toSec() const { return t_; }
  uint64_t toNSec() const { return static_cast<uint64_t>(t_ * 1.0e9); }
  bool isZero() const { return t_ == 0.0; }

  Duration operator-(const Time& o) const { return Duration(t_ - o.t_); }
  Time operator+(const Duration& d) const { return Time(t_ + d.t_); }
  Time operator-(const Duration& d) const { return Time(t_ - d.t_); }

  bool operator==(const Time& o) const { return t_ == o.t_; }
  bool operator!=(const Time& o) const { return t_ != o.t_; }
  bool operator<(const Time& o) const { return t_ < o.t_; }
  bool operator<=(const Time& o) const { return t_ <= o.t_; }
  bool operator>(const Time& o) const { return t_ > o.t_; }
  bool operator>=(const Time& o) const { return t_ >= o.t_; }
};

inline std::ostream& operator<<(std::ostream& os, const Time& t)
{
  return os << std::fixed << std::setprecision(9) << t.t_;
}

inline std::ostream& operator<<(std::ostream& os, const Duration& d)
{
  return os << std::fixed << std::setprecision(9) << d.t_;
}

inline const Time TIME_MIN{0.0};
inline const Time TIME_MAX{1.0e18};

using WallTime     = Time;
using WallDuration = Duration;

inline bool ok() { return true; }
}  // namespace ros
