#include "dumpGraph.h"

#include <ros/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <boost/filesystem.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>

/**
 * Function to save the graph, point clouds and gps data
 **/
void dump(const std::string& dump_directory,
  const gtsam::ISAM2& isam,
  const gtsam::Values& isam_current_estimate,
  const std::vector<double>& keyframe_stamps,
  const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& corner_cloud_keyframes,
  const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& surf_cloud_keyframes,
  const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& outlier_cloud_keyframes,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudKeyPoses3D,
  const pcl::PointCloud<PointTypePose>::Ptr& cloudKeyPoses6D,
  const std::vector<sensor_msgs::NavSatFix>& gps_data
) {
  boost::filesystem::create_directories(dump_directory);
  
  // Save the graph
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
  
  // Save the Point Clouds
  for(int i = 0; i < corner_cloud_keyframes.size(); i++) {
    std::string keyframe_directory = (boost::format("%s/%06d") % dump_directory % i).str();
    boost::filesystem::create_directories(keyframe_directory);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_corner(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_surf(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_outlier(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    *cloud_corner += *corner_cloud_keyframes[i];
    *cloud_surf += *surf_cloud_keyframes[i];
    *cloud_outlier += *outlier_cloud_keyframes[i];
    *cloud += *corner_cloud_keyframes[i];
    *cloud += *surf_cloud_keyframes[i];
    *cloud += *outlier_cloud_keyframes[i];

    Eigen::Isometry3f camera2lidar = Eigen::AngleAxisf(M_PI / 2.0f, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(M_PI / 2.0, Eigen::Vector3f::UnitY()) * Eigen::Isometry3f::Identity();

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
    
    pcl::transformPointCloud(*cloud, *transformed, camera2lidar);
    pcl::io::savePCDFileBinary(keyframe_directory + "/cloud.pcd", *transformed);

    // pcl::transformPointCloud(*cloud_corner, *transformed, camera2lidar);
    // cloud = transformPointCloud(cloud, keyframe_poses[i]);
    pcl::io::savePCDFileBinary(keyframe_directory + "/cloud_corner.pcd", *cloud_corner);

    // pcl::transformPointCloud(*cloud_surf, *transformed, camera2lidar);
    pcl::io::savePCDFileBinary(keyframe_directory + "/cloud_surf.pcd", *cloud_surf);

    // pcl::transformPointCloud(*cloud_outlier, *transformed, camera2lidar);
    pcl::io::savePCDFileBinary(keyframe_directory + "/cloud_outlier.pcd", *cloud_outlier);

    ros::Time stamp(keyframe_stamps[i]);

    std::ofstream data_ofs(keyframe_directory + "/data");
    data_ofs << "stamp " << stamp.sec << " " << stamp.nsec << "\n";
    data_ofs << "estimate\n" << keyframe_poses[i].matrix() << "\n";
    data_ofs << "odom\n" << keyframe_poses[i].matrix() << "\n";
    data_ofs << "accum_distance -1" << "\n";
    data_ofs << "id " << i << "\n";
  }

  pcl::io::savePCDFileBinary(dump_directory + "/cloudKeyPoses3D.pcd", *cloudKeyPoses3D);
  pcl::io::savePCDFileBinary(dump_directory + "/cloudKeyPoses6D.pcd", *cloudKeyPoses6D);
  
  // Save GPS Data
  std::ofstream gps_data_ofs(dump_directory + "/gps_data.txt");

  for(int i=0; i<gps_data.size(); i++){
    gps_data_ofs << gps_data[i].latitude << " " << gps_data[i].longitude << " " << gps_data[i].altitude << " ";
    gps_data_ofs << gps_data[i].position_covariance[0] << " " << gps_data[i].position_covariance[1] << " " << gps_data[i].position_covariance[2] << " " << gps_data[i].position_covariance[3] << " " << gps_data[i].position_covariance[4] << " " << gps_data[i].position_covariance[5] << " " << gps_data[i].position_covariance[6] << " " << gps_data[i].position_covariance[7] << " " << gps_data[i].position_covariance[8] << " "; 
    gps_data_ofs << "\n";
  }
}