#include "dumpGraph.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <boost/filesystem.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr transformPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, const gtsam::Pose3& transformIn) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZI>());

  double roll = transformIn.rotation().pitch();
  double pitch = transformIn.rotation().yaw();
  double yaw = transformIn.rotation().roll();

  pcl::PointXYZI* pointFrom;
  pcl::PointXYZI pointTo;

  int cloudSize = cloudIn->points.size();
  cloudOut->resize(cloudSize);

  for(int i = 0; i < cloudSize; ++i) {
    pointFrom = &cloudIn->points[i];
    float x1 = cos(yaw) * pointFrom->x - sin(yaw) * pointFrom->y;
    float y1 = sin(yaw) * pointFrom->x + cos(yaw) * pointFrom->y;
    float z1 = pointFrom->z;

    float x2 = x1;
    float y2 = cos(roll) * y1 - sin(roll) * z1;
    float z2 = sin(roll) * y1 + cos(roll) * z1;

    pointTo.x = cos(pitch) * x2 + sin(pitch) * z2 + transformIn.x();
    pointTo.y = y2 + transformIn.y();
    pointTo.z = -sin(pitch) * x2 + cos(pitch) * z2 + transformIn.z();
    pointTo.intensity = pointFrom->intensity;

    cloudOut->points[i] = pointTo;
  }

  cloudOut->width = cloudOut->size();
  cloudOut->height = 1;
  cloudOut->is_dense = false;

  return cloudOut;
}

void dump(const std::string& dump_directory, const gtsam::ISAM2& isam, const gtsam::Values& isam_current_estimate, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> corner_cloud_keyframes, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> surf_cloud_keyframes, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outlier_cloud_keyframes) {
  boost::filesystem::create_directories(dump_directory);

  std::vector<gtsam::Pose3> keyframe_poses(isam_current_estimate.size());

  std::ofstream graph_ofs(dump_directory + "/graph.g2o");
  for(const auto& vertex : isam_current_estimate) {
    Eigen::Matrix4d pose = vertex.value.cast<gtsam::Pose3>().matrix();
    keyframe_poses[vertex.key] = vertex.value.cast<gtsam::Pose3>();
    Eigen::Vector3d t = pose.block<3, 1>(0, 3) + Eigen::Vector3d::Random();
    Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
    graph_ofs << "VERTEX_SE3:QUAT " << vertex.key << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
  }
  graph_ofs << "FIX 0" << "\n";

  std::cout << "factors:" << isam.getFactorsUnsafe().size() << std::endl;
  for(const auto& factor_: isam.getFactorsUnsafe()) {
    auto between_factor = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor_);
    if(between_factor) {
      Eigen::Matrix4d relative = between_factor->measured().matrix();
      Eigen::Vector3d t = relative.block<3, 1>(0, 3);
      Eigen::Quaterniond q(relative.block<3, 3>(0, 0));

      graph_ofs << "EDGE_SE3:QUAT " << between_factor->key1() << " " << between_factor->key2();
      graph_ofs << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w();

      Eigen::VectorXd vars = 1.0 / between_factor->noiseModel()->sigmas().array();
      Eigen::MatrixXd inf = vars.asDiagonal();

      for(int i = 0; i < inf.rows(); i++) {
        for(int j=i; j<inf.cols(); j++) {
          graph_ofs << " " << inf(i, j);
        }
      }
      graph_ofs << "\n";
    }
  }

  for(int i = 0; i < corner_cloud_keyframes.size(); i++) {
    std::string keyframe_directory = (boost::format("%s/%06d") % dump_directory % i).str();
    boost::filesystem::create_directories(keyframe_directory);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    *cloud += *corner_cloud_keyframes[i];
    *cloud += *surf_cloud_keyframes[i];
    *cloud += *outlier_cloud_keyframes[i];

    Eigen::Isometry3f camera2lidar = Eigen::AngleAxisf(M_PI / 2.0f, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(M_PI / 2.0, Eigen::Vector3f::UnitY()) * Eigen::Isometry3f::Identity();

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*cloud, *transformed, camera2lidar);

    // cloud = transformPointCloud(cloud, keyframe_poses[i]);
    pcl::io::savePCDFileBinary(keyframe_directory + "/cloud.pcd", *transformed);

    std::ofstream data_ofs(keyframe_directory + "/data");
    data_ofs << "stamp 0 " << i << "\n";
    data_ofs << "estimate\n" << keyframe_poses[i].matrix() << "\n";
    data_ofs << "odom\n" << keyframe_poses[i].matrix() << "\n";
    data_ofs << "accum_distance -1" << "\n";
    data_ofs << "id " << i << "\n";
  }
}