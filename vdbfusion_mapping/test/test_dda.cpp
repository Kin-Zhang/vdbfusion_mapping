#include <iostream>
#include <string>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <openvdb/Types.h>
#include <openvdb/math/DDA.h>
#include <openvdb/math/Ray.h>
#include <openvdb/openvdb.h>

#include "timer.h"

float ComputeSDF(const Eigen::Vector3d &origin, const Eigen::Vector3d &point,
                 const Eigen::Vector3d &voxel_center) {
  const Eigen::Vector3d v_voxel_origin = voxel_center - origin;
  const Eigen::Vector3d v_point_voxel = point - voxel_center;
  const double dist = v_point_voxel.norm();
  const double proj = v_voxel_origin.dot(v_point_voxel);
  const double sign = proj / std::abs(proj);
  return static_cast<float>(sign * dist);
}

Eigen::Vector3d GetVoxelCenter(const openvdb::Coord &voxel,
                               const openvdb::math::Transform &xform) {
  const float voxel_size = xform.voxelSize()[0];
  openvdb::math::Vec3d v_wf = xform.indexToWorld(voxel) + voxel_size / 2.0;
  return Eigen::Vector3d(v_wf.x(), v_wf.y(), v_wf.z());
}

void generateSpherePoint(std::vector<Eigen::Vector3d> &points) {
  double radius_ball = 10.0;
  for (double phi = 0.0; phi <= M_PI; phi += M_PI / 360.0) {
    double radius_circle = radius_ball * sin(phi);
    double z = radius_ball * cos(phi);
    for (double theta = 0.0; theta <= 2 * M_PI; theta += M_PI / 360.0) {
      double x = radius_circle * cos(theta);
      double y = radius_circle * sin(theta);
      points.push_back(Eigen::Vector3d(x, y, z));
    }
  }

  radius_ball /= 2.0;
  for (double phi = 0.0; phi <= M_PI; phi += M_PI / 360.0) {
    double radius_circle = radius_ball * sin(phi);
    double z = radius_ball * cos(phi);
    for (double theta = 0.0; theta <= 2 * M_PI; theta += M_PI / 360.0) {
      double x = radius_circle * cos(theta);
      double y = radius_circle * sin(theta);
      points.push_back(Eigen::Vector3d(x, y, z));
    }
  }
}

int main(int argc, char *argv[]) { 
  TIC;
  openvdb::initialize();

  // Create an empty floating-point grid with background value 0.
  float sdf_trunc = 0.2;
  float voxel_size = 0.1;
  bool space_carving = false;

  openvdb::FloatGrid::Ptr tsdf = openvdb::FloatGrid::create(sdf_trunc);
  // Name the grid .
  tsdf->setName("sdf_distance");
  // Associate a scaling transform with the grid that sets the voxel size
  tsdf->setTransform(openvdb::math::Transform::createLinearTransform(
      /*voxel size=*/voxel_size));
  tsdf->setGridClass(openvdb::GRID_LEVEL_SET);

  openvdb::FloatGrid::Ptr weights = openvdb::FloatGrid::create(0.0f);
  weights->setName("sdf_weight");
  weights->setTransform(
      openvdb::math::Transform::createLinearTransform(voxel_size));
  weights->setGridClass(openvdb::GRID_UNKNOWN);

  // insert a point
  std::vector<Eigen::Vector3d> points;
  // points.push_back(Eigen::Vector3d(10.0, 10.0, 0.0));
  generateSpherePoint(points);
  {
    Eigen::Vector3d origin(0.0, 0.0, 0.0);
    const openvdb::math::Transform &xform = tsdf->transform();
    const openvdb::Vec3R eye(origin.x(), origin.y(), origin.z());
    auto tsdf_acc = tsdf->getAccessor();
    auto weights_acc = weights->getAccessor();

    size_t cnt = 0;
    TRE;
    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
      const Eigen::Vector3d direction = point - origin;
      openvdb::Vec3R dir(direction.x(), direction.y(), direction.z());
      dir.normalize();
      const auto depth = static_cast<float>(direction.norm());
      const float t0 = depth - sdf_trunc;
      const float t1 = depth + sdf_trunc;
      const auto ray =
          openvdb::math::Ray<float>(eye, dir, t0, t1).worldToIndex(*tsdf);
      openvdb::math::DDA<decltype(ray)> dda(ray);
      do {
        const auto voxel = dda.voxel();
        const auto voxel_center = GetVoxelCenter(voxel, xform);
        const auto sdf = ComputeSDF(origin, point, voxel_center);
        if (sdf > -sdf_trunc) {
          // std::cout << "inserted voxel: " << voxel << std::endl;
          const float tsdf = std::min(sdf_trunc, sdf);
          const float weight = 1.0;
          const float last_weight = weights_acc.getValue(voxel);
          const float last_tsdf = tsdf_acc.getValue(voxel);
          const float new_weight = weight + last_weight;
          const float new_tsdf =
              (last_tsdf * last_weight + tsdf * weight) / (new_weight);
          tsdf_acc.setValue(voxel, new_tsdf);       // tsdf update
          weights_acc.setValue(voxel, new_weight);  // weight update
          if (last_weight < 0.5) cnt++;
        }
      } while (dda.step());
    });
    TOC("inserting points costs", true);
    std::cout << "number of active voxels: " << cnt << std::endl;
  }

  // search active voxel
  {
    Eigen::Vector3d origin(0.0, 0.0, 0.0);
    const openvdb::math::Transform &xform = tsdf->transform();
    const openvdb::Vec3R eye(origin.x(), origin.y(), origin.z());
    auto tsdf_acc = tsdf->getAccessor();
    auto weights_acc = weights->getAccessor();

    TRE;
    size_t cnt = 0;
    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
      const Eigen::Vector3d direction = point - origin;
      openvdb::Vec3R dir(direction.x(), direction.y(), direction.z());
      dir.normalize();
      const auto depth = static_cast<float>(direction.norm());
      // const float t0 = space_carving ? 0.0f : depth - sdf_trunc;
      const float t0 = 0.0f;
      const float t1 = depth - sdf_trunc;
      // std::cout << "t0, t1: " << t0 << ", " << t1 << std::endl;

      openvdb::math::Ray<float> ray =
          openvdb::math::Ray<float>(eye, dir, t0, t1).worldToIndex(*tsdf);
      // std::cout << "ray (t0, t1): " << ray.t0() << ", " << ray.t1()
      //           << std::endl;
      openvdb::math::DDA<decltype(ray)> dda(ray);
      do {
        const openvdb::Coord voxel = dda.voxel();
        if (tsdf_acc.isValueOn(voxel)) {
          // std::cout << "active voxel: " << voxel << std::endl;
          // dda.print();
          cnt++;
        }
      } while (dda.step());
    });
    TOC("DDA searching active voxel", true);
    std::cout << "number of active voxels (repeated counted): " << cnt << std::endl;
  }

  // ********* HDDA
  // using GridType = openvdb::FloatGrid;
  // using TreeType = GridType::TreeType;
  // using RootType = TreeType::RootNodeType;  // level 3 RootNode
  // assert(RootType::LEVEL == 3);
  // using Int1Type = RootType::ChildNodeType;  // level 2 InternalNode
  // using Int2Type = Int1Type::ChildNodeType;  // level 1 InternalNode
  // using LeafType = TreeType::LeafNodeType;   // level 0 LeafNode
  {
    Eigen::Vector3d origin(0.0, 0.0, 0.0);
    const openvdb::math::Transform &xform = tsdf->transform();
    const openvdb::Vec3R eye(origin.x(), origin.y(), origin.z());
    auto tsdf_acc = tsdf->getAccessor();
    auto weights_acc = weights->getAccessor();

    size_t cnt = 0;
    TRE;
    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
      const Eigen::Vector3d direction = point - origin;
      openvdb::Vec3R dir(direction.x(), direction.y(), direction.z());
      dir.normalize();
      const auto depth = static_cast<float>(direction.norm());
      const float t0 = 0.0f;
      const float t1 = depth - sdf_trunc;

      openvdb::math::Ray<float> ray =
          openvdb::math::Ray<float>(eye, dir, t0, t1).worldToIndex(*tsdf);
      openvdb::math::VolumeHDDA<openvdb::FloatGrid::TreeType, decltype(ray), 0>
          volume_hdda;
      std::vector<openvdb::math::Ray<float>::TimeSpan> times;
      volume_hdda.hits(ray, tsdf_acc, times);
      for (const auto &t : times) {
        openvdb::math::Ray<float> ray_sub =
            openvdb::math::Ray<float>(eye, dir, t0, t1).worldToIndex(*tsdf);
        ray_sub.setTimes(t.t0, t.t1);
        openvdb::math::DDA<decltype(ray_sub)> dda(ray_sub);
        do {
          const openvdb::Coord voxel = dda.voxel();
          if (tsdf_acc.isValueOn(voxel)) {
            cnt++;
          }
        } while (dda.step());
      }
    });
    TOC("HDDA searching active voxel: ", true);
    std::cout << "number of active voxels (repeated counted): " << cnt << std::endl;
  }

  {
    openvdb::io::File file("/tmp/test_dda.vdb");
    openvdb::GridPtrVec grids;
    grids.push_back(tsdf);
    file.write(grids);
    file.close();
  }

  // std::cout << "Testing random access:" << std::endl;
  // // Get an accessor for coordinate-based access to voxels.
  // openvdb::FloatGrid::Accessor accessor = grid->getAccessor();
  // // Define a coordinate with large signed indices.
  // openvdb::Coord xyz(1000, -200000000, 30000000);
  // // Set the voxel value at (1000, -200000000, 30000000) to 1.
  // accessor.setValue(xyz, 1.0);
  // // Verify that the voxel value at (1000, -200000000, 30000000) is 1.
  // std::cout << "Grid" << xyz << " = " << accessor.getValue(xyz) << std::endl;
  // // Reset the coordinates to those of a different voxel.
  // xyz.reset(1000, 200000000, -30000000);
  // // Verify that the voxel value at (1000, 200000000, -30000000) is
  // // the background value, 0.
  // std::cout << "Grid" << xyz << " = " << accessor.getValue(xyz) << std::endl;
  // // Set the voxel value at (1000, 200000000, -30000000) to 2.
  // accessor.setValue(xyz, 2.0);
  // // Set the voxels at the two extremes of the available coordinate space.
  // // For 32-bit signed coordinates these are (-2147483648, -2147483648,
  // // -2147483648) and (2147483647, 2147483647, 2147483647).
  // accessor.setValue(openvdb::Coord::min(), 3.0f);
  // accessor.setValue(openvdb::Coord::max(), 4.0f);

  // TRE;
  // std::cout << "Testing sequential access:" << std::endl;
  // // Print all active ("on") voxels by means of an iterator.
  // for (openvdb::FloatGrid::ValueOnCIter iter = grid->cbeginValueOn(); iter;
  //      ++iter) {
  //   std::cout << "Grid" << iter.getCoord() << " = " << *iter << std::endl;
  // }
  // std::cout << "Loading grids costs: " << tt.toc() << " ms" << std::endl;

  return 0;
}
