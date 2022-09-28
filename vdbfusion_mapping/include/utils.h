#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <pcl/point_cloud.h>
namespace common {

// clang-format off
class WeightFunction {
    public:
    inline static float tsdf_ = 0.0f;
    inline static float epsilon_ = 0.0f;

    inline static auto constant_weight = [](float sdf) { return 1.0f; };

    inline static auto linear_weight = [](float sdf) {
    static float a = 0.5f * tsdf_ / (tsdf_ - epsilon_);
    static float b = 0.5f         / (tsdf_ - epsilon_);
    if (sdf < epsilon_) return 1.0f;
    if ((epsilon_ <= sdf) && (sdf <= tsdf_))
        return (a - b * sdf  + 0.5f);
    return 0.5f;
    };

    inline static auto exp_weight = [](float sdf) {
    if (sdf < epsilon_) return 1.0f;
    if ((epsilon_ <= sdf) && (sdf <= tsdf_))
        return float(exp(-tsdf_ * (sdf - epsilon_) * (sdf - epsilon_)));
    return float(exp(-tsdf_ * (tsdf_ - epsilon_) * (tsdf_ - epsilon_)));
    };
};
// clang-format on

// Check if all coordinates in the PCL point are finite.
template <typename PointType>
inline bool isPointFinite(const PointType &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

template <typename Scalar, typename PointType>
void PCL2Eigen(const pcl::PointCloud<PointType> &ptcloud_pcl,
               std::vector<Eigen::Matrix<Scalar, 3, 1>> &ptcloud_eig) {
  ptcloud_eig.clear();
  ptcloud_eig.reserve(ptcloud_pcl.size());
  for (const auto point : ptcloud_pcl) {
    if (!isPointFinite(point))
      continue;
    Eigen::Matrix<Scalar, 3, 1> pt;
    pt(0) = static_cast<Scalar>(point.x);
    pt(1) = static_cast<Scalar>(point.y);
    pt(2) = static_cast<Scalar>(point.z);
    ptcloud_eig.push_back(pt);
  }
}

class ThreadSafeIndex {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // NOTE: The ThreadSafeIndex base destructor must be marked virtual.
  //       Otherwise the destructors of derived classes don't get called when
  //       derived class instances are destructed through base class pointers.
  //       This would result leaking memory due to derived class member
  //       variables not being freed.
  virtual ~ThreadSafeIndex() = default;

  /// returns true if index is valid, false otherwise
  bool getNextIndex(size_t* idx);

  void reset();

 protected:
  explicit ThreadSafeIndex(size_t number_of_points);

  virtual size_t getNextIndexImpl(size_t base_idx) = 0;

  std::atomic<size_t> atomic_idx_;
  const size_t number_of_points_;
};

} // namespace common

#endif // UTILS_H