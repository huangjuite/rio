// ROS-free stand-in for <pcl/point_cloud.h>.
//
// Only the operations the upstream estimator performs are provided:
// construction, `size()`, `at()`, `push_back()` and iteration.

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/StdVector>

namespace pcl
{
template <typename PointT>
struct PointCloud
{
  using VectorType  = std::vector<PointT, Eigen::aligned_allocator<PointT>>;
  using Ptr         = std::shared_ptr<PointCloud<PointT>>;
  using ConstPtr    = std::shared_ptr<const PointCloud<PointT>>;
  using iterator    = typename VectorType::iterator;
  using const_iterator = typename VectorType::const_iterator;

  VectorType points;
  uint32_t width  = 0;
  uint32_t height = 1;
  bool is_dense   = true;

  std::size_t size() const { return points.size(); }
  bool empty() const { return points.empty(); }
  void clear() { points.clear(); width = 0; }
  void reserve(std::size_t n) { points.reserve(n); }
  void resize(std::size_t n) { points.resize(n); width = static_cast<uint32_t>(n); }

  void push_back(const PointT& p)
  {
    points.push_back(p);
    width = static_cast<uint32_t>(points.size());
  }

  PointT& at(std::size_t i) { return points.at(i); }
  const PointT& at(std::size_t i) const { return points.at(i); }
  PointT& operator[](std::size_t i) { return points[i]; }
  const PointT& operator[](std::size_t i) const { return points[i]; }

  iterator begin() { return points.begin(); }
  iterator end() { return points.end(); }
  const_iterator begin() const { return points.begin(); }
  const_iterator end() const { return points.end(); }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pcl
