// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar
//   Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems
//      (IROS). October 2018.

#include "mapOptimization.h"
#include <future>

using namespace gtsam;

MapOptimization::MapOptimization(ros::NodeHandle &node,
                                 Channel<AssociationOut> &input_channel)
    : nh(node),
      _input_channel(input_channel)
{
  downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
  downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
  downSizeFilterOutlier.setLeafSize(0.4, 0.4, 0.4);

  allocateMemory();

  subVehicleOdom = nh.subscribe<nav_msgs::Odometry>("/vehicle_odom", 10, &MapOptimization::subVehicleOdomHandler, this);
  _run_thread = std::thread(&MapOptimization::run, this);

  cloudKeyPose3DSize=0;

  boost::filesystem::create_directories("/tmp/dump");
}

MapOptimization::~MapOptimization()
{
  _input_channel.send({});
  _run_thread.join();

}

void MapOptimization::allocateMemory() {
  cloudKeyPoses3DGlobal.reset(new pcl::PointCloud<PointType>());
  cloudKeyPoses6DGlobal.reset(new pcl::PointCloud<PointTypePose>());

  cloudKeyPoses3DTruth.reset(new pcl::PointCloud<PointType>());
  cloudKeyPoses6DTruth.reset(new pcl::PointCloud<PointTypePose>());

  surroundingKeyPoses.reset(new pcl::PointCloud<PointType>());
  surroundingKeyPosesDS.reset(new pcl::PointCloud<PointType>());

  laserCloudCornerLast.reset(
      new pcl::PointCloud<PointType>());  // corner feature set from
                                          // odoOptimization
  laserCloudSurfLast.reset(
      new pcl::PointCloud<PointType>());  // surf feature set from
                                          // odoOptimization
  laserCloudCornerLastDS.reset(
      new pcl::PointCloud<PointType>());  // downsampled corner featuer set
                                          // from odoOptimization
  laserCloudSurfLastDS.reset(
      new pcl::PointCloud<PointType>());  // downsampled surf featuer set from
                                          // odoOptimization
  laserCloudOutlierLast.reset(
      new pcl::PointCloud<PointType>());  // corner feature set from
                                          // odoOptimization
  laserCloudOutlierLastDS.reset(
      new pcl::PointCloud<PointType>());  // downsampled corner feature set
                                          // from odoOptimization
  laserCloudSurfTotalLast.reset(
      new pcl::PointCloud<PointType>());  // surf feature set from
                                          // odoOptimization
  laserCloudSurfTotalLastDS.reset(
      new pcl::PointCloud<PointType>());  // downsampled surf featuer set from
                                          // odoOptimization

  timeLaserOdometry = 0;
  timeLastGloalMapPublish = 0;
  timeLastProcessing = -1;

  for (int i = 0; i < 6; ++i) {
    transformLast[i] = 0;
    transformSum[i] = 0;
    transformIncre[i] = 0;
    transformTobeMapped[i] = 0;
    transformBefMapped[i] = 0;
    transformAftMapped[i] = 0;
  }
  laserCloudCornerLastDSNum = 0;
  laserCloudSurfLastDSNum = 0;
  laserCloudOutlierLastDSNum = 0;
  laserCloudSurfTotalLastDSNum = 0;

  fullCloudMap.reset(new pcl::PointCloud<PointType>());
}

pcl::PointCloud<PointType>::Ptr MapOptimization::transformPointCloud(
    pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn) {
  pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

  PointType *pointFrom;
  PointType pointTo;

  int cloudSize = cloudIn->points.size();
  cloudOut->resize(cloudSize);

  for (int i = 0; i < cloudSize; ++i) {
    pointFrom = &cloudIn->points[i];
    float x1 = cos(transformIn->yaw) * pointFrom->x -
               sin(transformIn->yaw) * pointFrom->y;
    float y1 = sin(transformIn->yaw) * pointFrom->x +
               cos(transformIn->yaw) * pointFrom->y;
    float z1 = pointFrom->z;

    float x2 = x1;
    float y2 = cos(transformIn->roll) * y1 - sin(transformIn->roll) * z1;
    float z2 = sin(transformIn->roll) * y1 + cos(transformIn->roll) * z1;

    pointTo.x = cos(transformIn->pitch) * x2 + sin(transformIn->pitch) * z2 +
                transformIn->x;
    pointTo.y = y2 + transformIn->y;
    pointTo.z = -sin(transformIn->pitch) * x2 + cos(transformIn->pitch) * z2 +
                transformIn->z;
    pointTo.intensity = pointFrom->intensity;

    cloudOut->points[i] = pointTo;
  }
  return cloudOut;
}

void MapOptimization::downsampleCurrentScan() {
  laserCloudCornerLastDS->clear();
  downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
  downSizeFilterCorner.filter(*laserCloudCornerLastDS);
  laserCloudCornerLastDSNum = laserCloudCornerLastDS->points.size();

  laserCloudSurfLastDS->clear();
  downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
  downSizeFilterSurf.filter(*laserCloudSurfLastDS);
  laserCloudSurfLastDSNum = laserCloudSurfLastDS->points.size();

  laserCloudOutlierLastDS->clear();
  downSizeFilterOutlier.setInputCloud(laserCloudOutlierLast);
  downSizeFilterOutlier.filter(*laserCloudOutlierLastDS);
  laserCloudOutlierLastDSNum = laserCloudOutlierLastDS->points.size();

  laserCloudSurfTotalLast->clear();
  laserCloudSurfTotalLastDS->clear();
  *laserCloudSurfTotalLast += *laserCloudSurfLastDS;
  *laserCloudSurfTotalLast += *laserCloudOutlierLastDS;
  downSizeFilterSurf.setInputCloud(laserCloudSurfTotalLast);
  downSizeFilterSurf.filter(*laserCloudSurfTotalLastDS);
  laserCloudSurfTotalLastDSNum = laserCloudSurfTotalLastDS->points.size();
}

void MapOptimization::run() {
  size_t cycle_count = 0;

  while (ros::ok()) {
    AssociationOut association;
    _input_channel.receive(association);
    if( !ros::ok() ) break;

    {
      std::lock_guard<std::mutex> lock(mtx);

      laserCloudCornerLast = association.cloud_corner_last;
      laserCloudSurfLast = association.cloud_surf_last;
      laserCloudOutlierLast = association.cloud_outlier_last;

      timeLaserOdometry = association.laser_odometry.header.stamp.toSec();
      timeLastProcessing = timeLaserOdometry;

      // OdometryToTransform(association.laser_odometry, transformSum);

      downsampleCurrentScan();
      
      saveGroundTruth();
    }
    cycle_count++;
  }
  pcl::io::savePCDFileBinary("/tmp/dump/cloudKeyPoses3D.pcd", *cloudKeyPoses3DTruth);
  pcl::io::savePCDFileBinary("/tmp/dump/cloudKeyPoses6D.pcd", *cloudKeyPoses6DTruth);
  pcl::io::savePCDFileBinary("/tmp/dump/fullCloud.pcd", *fullCloudMap);
}

bool MapOptimization::doWeSave(nav_msgs::Odometry true_transform){

  currentRobotPosPoint.x = true_transform.pose.pose.position.x;
  currentRobotPosPoint.y = true_transform.pose.pose.position.y;
  currentRobotPosPoint.z = true_transform.pose.pose.position.z;

  bool saveThisKeyFrame = true;
  if (sqrt((previousRobotPosPoint.x - currentRobotPosPoint.x) *
               (previousRobotPosPoint.x - currentRobotPosPoint.x) +
           (previousRobotPosPoint.y - currentRobotPosPoint.y) *
               (previousRobotPosPoint.y - currentRobotPosPoint.y) +
           (previousRobotPosPoint.z - currentRobotPosPoint.z) *
               (previousRobotPosPoint.z - currentRobotPosPoint.z)) < 0.3) {
    saveThisKeyFrame = false;
  }

  previousRobotPosPoint = currentRobotPosPoint;

  if (saveThisKeyFrame == false && !(cloudKeyPose3DSize==0)) return false;

  return true;
  
}

void MapOptimization::saveGroundTruth(){
  std::string carFrame="vehicle/085";
  std::string worldFrame="map";

  tf::TransformListener listener;

  PointType thisPose3D;
  PointTypePose thisPose6D;

  try{

    if(doWeSave(tempVehicleOdomMsg)){
      std::string keyframe_directory = (boost::format("/tmp/dump/%06d") % cloudKeyPose3DSize).str();
      boost::filesystem::create_directories(keyframe_directory);

      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_corner(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_surf(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_outlier(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

      *cloud_corner+=*laserCloudCornerLastDS;
      *cloud_surf+=*laserCloudSurfLastDS;
      *cloud_outlier+=*laserCloudOutlierLastDS;

      *cloud+=*laserCloudCornerLastDS;
      *cloud+=*laserCloudSurfLastDS;
      *cloud+=*laserCloudOutlierLastDS;

      pcl::io::savePCDFileBinary(keyframe_directory + "/cloud.pcd", *cloud);
      pcl::io::savePCDFileBinary(keyframe_directory + "/cloud_corner.pcd", *cloud_corner);
      pcl::io::savePCDFileBinary(keyframe_directory + "/cloud_surf.pcd", *cloud_surf);
      pcl::io::savePCDFileBinary(keyframe_directory + "/cloud_outlier.pcd", *cloud_outlier);

      thisPose3D.x=tempVehicleOdomMsg.pose.pose.position.y;
      thisPose3D.y=tempVehicleOdomMsg.pose.pose.position.z;
      thisPose3D.z=tempVehicleOdomMsg.pose.pose.position.x;
      thisPose3D.intensity=cloudKeyPose3DSize;

      cloudKeyPoses3DTruth->push_back(thisPose3D);
      cloudKeyPose3DSize+=1;
      thisPose6D.x=thisPose3D.x;
      thisPose6D.y=thisPose3D.y;
      thisPose6D.z=thisPose3D.z;
      thisPose3D.intensity=thisPose3D.intensity;
      double w,x,y,z;

      x=tempVehicleOdomMsg.pose.pose.orientation.x;
      y=tempVehicleOdomMsg.pose.pose.orientation.y;
      z=tempVehicleOdomMsg.pose.pose.orientation.z;
      w=tempVehicleOdomMsg.pose.pose.orientation.w;

      double true_pitch,true_roll,true_yaw;
      tf::Matrix3x3(tf::Quaternion(x,y,z,w)).getRPY(true_roll,true_pitch,true_yaw);
      thisPose6D.pitch=true_yaw;
      thisPose6D.yaw=true_roll;
      thisPose6D.roll=true_pitch;
      thisPose6D.time = timeLaserOdometry;
      cloudKeyPoses6DTruth->push_back(thisPose6D);


      Eigen::Isometry3f camera2lidar = Eigen::AngleAxisf(M_PI / 2.0f, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(M_PI / 2.0, Eigen::Vector3f::UnitY()) * Eigen::Isometry3f::Identity();
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
      *cloud_temp = *transformPointCloud(cloud, &thisPose6D);
      pcl::transformPointCloud(*cloud_temp, *transformed, camera2lidar);

      *fullCloudMap += *transformed;

      gtsam::Rot3 rot(w,x,y,z);
      gtsam::Point3 t(thisPose3D.x,thisPose3D.y,thisPose3D.z);
      gtsam::Pose3 true_pose(rot,t);

      ros::Time stamp(timeLaserOdometry);

      std::ofstream data_ofs(keyframe_directory + "/data");
      data_ofs << "stamp " << stamp.sec << " " << stamp.nsec << "\n";
      data_ofs << "estimate\n" << true_pose.matrix() << "\n";
      data_ofs << "odom\n" << true_pose.matrix() << "\n";
      data_ofs << "accum_distance -1" << "\n";
      data_ofs << "id " << cloudKeyPose3DSize << "\n";
    }
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
}

void MapOptimization::subVehicleOdomHandler(const nav_msgs::Odometry::ConstPtr& msg){
  tempVehicleOdomMsg = (*msg);
}