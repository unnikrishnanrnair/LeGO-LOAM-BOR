#ifndef MAPOPTIMIZATION_H
#define MAPOPTIMIZATION_H

#include "lego_loam/utility.h"
#include "lego_loam/channel.h"
#include "lego_loam/nanoflann_pcl.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <fstream>
#include <tf/transform_listener.h>

inline gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint) {
  // camera frame to lidar frame
  return gtsam::Pose3(
      gtsam::Rot3::RzRyRx(double(thisPoint.yaw), double(thisPoint.roll),
                          double(thisPoint.pitch)),
      gtsam::Point3(double(thisPoint.z), double(thisPoint.x),
                    double(thisPoint.y)));
}

inline Eigen::Affine3f pclPointToAffine3fCameraToLidar(
    PointTypePose thisPoint) {
  // camera frame to lidar frame
  return pcl::getTransformation(thisPoint.z, thisPoint.x, thisPoint.y,
                                thisPoint.yaw, thisPoint.roll, thisPoint.pitch);
}


class MapOptimization {

 public:
  /**
  * Default Constructor
  */
  MapOptimization(ros::NodeHandle& node, Channel<AssociationOut> &input_channel);

  ~MapOptimization();

  /**
  * Main Function
  */
  void run();

  /**
  * Handler for odometry data
  */ 
  void subVehicleOdomHandler(const nav_msgs::Odometry::ConstPtr& msg);

 private:

  ros::NodeHandle& nh;

  int cloudKeyPose3DSize;

  Channel<AssociationOut>& _input_channel;
  std::thread _run_thread;

  PointType previousRobotPosPoint;
  PointType currentRobotPosPoint;

  pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
  pcl::PointCloud<PointType>::Ptr cloudKeyPoses3DGlobal;
  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6DGlobal;
  pcl::PointCloud<PointType>::Ptr cloudKeyPoses3DTruth;
  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6DTruth;

  pcl::PointCloud<PointType>::Ptr surroundingKeyPoses;
  pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS;

  pcl::PointCloud<PointType>::Ptr
      laserCloudCornerLast;  // corner feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudSurfLast;  // surf feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudCornerLastDS;  // downsampled corner featuer set from
      // odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudSurfLastDS;  // downsampled surf featuer set from
      // odoOptimization

  pcl::PointCloud<PointType>::Ptr
      laserCloudOutlierLast;  // corner feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudOutlierLastDS;  // corner feature set from odoOptimization

  pcl::PointCloud<PointType>::Ptr
      laserCloudSurfTotalLast;  // surf feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudSurfTotalLastDS;  // downsampled corner featuer set from
      // odoOptimization


  pcl::VoxelGrid<PointType> downSizeFilterCorner;
  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  pcl::VoxelGrid<PointType> downSizeFilterOutlier;

  double timeLaserOdometry;
  double timeLastGloalMapPublish;

  float transformLast[6];
  float transformSum[6];     // the odometry data
  float transformIncre[6];
  float transformTobeMapped[6];   // transform to be mapped
  float transformBefMapped[6];    // the previous odometry values
  float transformAftMapped[6];    // transform after optimisation 

  std::mutex mtx;

  double timeLastProcessing;

  bool isDegenerate;

  int laserCloudCornerLastDSNum;
  int laserCloudSurfLastDSNum;
  int laserCloudOutlierLastDSNum;
  int laserCloudSurfTotalLastDSNum;

  //ros::Subscriber subVehicleOdom;
  nav_msgs::Odometry tempVehicleOdomMsg;

  pcl::PointCloud<PointType>::Ptr fullCloudMap;

 private:
  /**
  * Initialise the point clouds and other variables
  */
  void allocateMemory();
  
  /**
  * Function to apply a transform to a given point cloud. 
  */
  pcl::PointCloud<PointType>::Ptr transformPointCloud(
      pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);
  /**
  * Down sample the point cloud using down sizing filters
  */
  void downsampleCurrentScan();

  /**
  * Save the carla ground truth data
  */ 
  void saveGroundTruth();
  /**
  * Function to check whether the car has moved enough to save the new frame
  */ 
  bool doWeSave(nav_msgs::Odometry true_transform);
};

#endif // MAPOPTIMIZATION_H
