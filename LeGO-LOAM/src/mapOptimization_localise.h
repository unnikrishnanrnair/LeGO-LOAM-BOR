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

 private:
  gtsam::NonlinearFactorGraph gtSAMgraph;
  gtsam::Values initialEstimate;
  gtsam::Values optimizedEstimate;
  gtsam::ISAM2 *isam;
  gtsam::Values isamCurrentEstimate;

  ros::NodeHandle& nh;
  // get these values from ros params  
  bool _loop_closure_enabled;
  float _scan_period;
  float _noise_scale_trans;
  float _noise_scale_rot;

  // get these values too from ros params   
  float _surrounding_keyframe_search_radius;
  int   _surrounding_keyframe_search_num;
  float _history_keyframe_search_radius;
  int   _history_keyframe_search_num;
  float _history_keyframe_fitness_score;
  float _global_map_visualization_search_radius;

  int cloudKeyPose3DSize; // keep count of numbers of frames covered

  Channel<AssociationOut>& _input_channel;
  std::thread _run_thread;

  Channel<bool> _publish_global_signal;
  std::thread _publish_global_thread;
  void publishGlobalMapThread();

  bool globalMapPublished;

  //Channel<bool> _loop_closure_signal;
  // std::thread _loop_closure_thread;
  // void loopClosureThread();

  ros::Publisher pubLaserCloudSurround;
  ros::Publisher pubOdomAftMapped;
  ros::Publisher pubKeyPoses;

  ros::Publisher pubHistoryKeyFrames;
  ros::Publisher pubIcpKeyFrames;
  ros::Publisher pubRecentKeyFrames;

  nav_msgs::Odometry odomAftMapped;
  tf::StampedTransform aftMappedTrans;
  tf::TransformBroadcaster tfBroadcaster;

//   std::vector<double> keyframeStamps;
//   std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
//   std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
//   std::vector<pcl::PointCloud<PointType>::Ptr> outlierCloudKeyFrames;

  std::deque<pcl::PointCloud<PointType>::Ptr> recentCornerCloudKeyFrames;
  std::deque<pcl::PointCloud<PointType>::Ptr> recentSurfCloudKeyFrames;
  std::deque<pcl::PointCloud<PointType>::Ptr> recentOutlierCloudKeyFrames;
  int latestFrameID;

  std::vector<int> surroundingExistingKeyPosesID;
  std::deque<pcl::PointCloud<PointType>::Ptr> surroundingCornerCloudKeyFrames;
  std::deque<pcl::PointCloud<PointType>::Ptr> surroundingSurfCloudKeyFrames;
  std::deque<pcl::PointCloud<PointType>::Ptr> surroundingOutlierCloudKeyFrames;

  PointType previousRobotPosPoint;
  PointType currentRobotPosPoint;

//   pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
//   pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
  pcl::PointCloud<PointType>::Ptr cloudKeyPoses3DGlobal;
  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6DGlobal;

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

  pcl::PointCloud<PointType>::Ptr laserCloudOri;
  pcl::PointCloud<PointType>::Ptr coeffSel;

  pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
  pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

  nanoflann::KdTreeFLANN<PointType> kdtreeCornerFromMap;
  nanoflann::KdTreeFLANN<PointType> kdtreeSurfFromMap;

  nanoflann::KdTreeFLANN<PointType> kdtreeSurroundingKeyPoses;
  nanoflann::KdTreeFLANN<PointType> kdtreeHistoryKeyPoses;

  pcl::PointCloud<PointType>::Ptr nearHistoryCornerKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr nearHistoryCornerKeyFrameCloudDS;
  pcl::PointCloud<PointType>::Ptr nearHistorySurfKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr nearHistorySurfKeyFrameCloudDS;

  pcl::PointCloud<PointType>::Ptr latestCornerKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloudDS;

  nanoflann::KdTreeFLANN<PointType> kdtreeGlobalMap;
  pcl::PointCloud<PointType>::Ptr globalMapKeyPoses;
  pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS;
  pcl::PointCloud<PointType>::Ptr globalMapKeyFrames;
  pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS;

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  pcl::VoxelGrid<PointType> downSizeFilterCorner;
  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  pcl::VoxelGrid<PointType> downSizeFilterOutlier;
  pcl::VoxelGrid<PointType>
      downSizeFilterHistoryKeyFrames;  // for history key frames of loop closure
  pcl::VoxelGrid<PointType>
      downSizeFilterSurroundingKeyPoses;  // for surrounding key poses of
      // scan-to-map optimization
  pcl::VoxelGrid<PointType>
      downSizeFilterGlobalMapKeyPoses;  // for global map visualization
  pcl::VoxelGrid<PointType>
      downSizeFilterGlobalMapKeyFrames;  // for global map visualization

  double timeLaserOdometry;
  double timeLastGloalMapPublish;

  float transformLast[6];
  float transformSum[6]; // the odometry values 
  float transformIncre[6];
  float transformTobeMapped[6];  // transform to be mapped
  float transformBefMapped[6];   // the previous odometry values
  float transformAftMapped[6];   // transform after optimisation


  std::mutex mtx;

  double timeLastProcessing;

  PointType pointOri, pointSel, pointProj, coeff;

  Eigen::Matrix<float, 5, 3> matA0;
  Eigen::Matrix<float, 5, 1> matB0;
  Eigen::Vector3f matX0;

  Eigen::Matrix3f matA1;
  Eigen::Matrix<float, 1, 3> matD1;
  Eigen::Matrix3f matV1;

  Eigen::Matrix<float, 6, 6> matP;

  bool isDegenerate;

  int laserCloudCornerFromMapDSNum;
  int laserCloudSurfFromMapDSNum;
  int laserCloudCornerLastDSNum;
  int laserCloudSurfLastDSNum;
  int laserCloudOutlierLastDSNum;
  int laserCloudSurfTotalLastDSNum;

  bool potentialLoopFlag;
  double timeSaveFirstCurrentScanForLoopClosure;
  int closestHistoryFrameID;
  int latestFrameIDLoopCloure;

  bool aLoopIsClosed;
  
  // wrt transformToBeMapped 
  float cRoll, sRoll, cPitch, sPitch, cYaw, sYaw, tX, tY, tZ;
  // wrt given transform
  float ctRoll, stRoll, ctPitch, stPitch, ctYaw, stYaw, tInX, tInY, tInZ;

 private:
  /**
   * Initialise all point clouds, transfrom arrays
   * and other variables.
   * */
  void allocateMemory();
  /**
   * Use TransformSum, TransformBefMapped and TransformAftMapped
   * to calculate TransformToBeMapped wrt this Map
   * */
  void transformAssociateToMap();
  /**
   * Update TransformAftMapped to value after LM optimisation
   * Make TransformBefMapped equal to TransformSum
   **/ 
  void transformUpdate();
  /**
   * Update cRoll, sRoll, cPitch, sPitch, cYaw, sYaw, tX, tY, tZ
   * using TransformToBeMapped 
   **/ 
  void updatePointAssociateToMapSinCos();
  /**
   * Find the co-ordinate of pi wrt map and save in po
   **/ 
  void pointAssociateToMap(PointType const *const pi, PointType *const po);
  
  /**
   * Update ctRoll, stRoll, ctPitch, stPitch, ctYaw, stYaw, tInX, 
   * tInY, tInZ using the Transform
   **/ 
  void updateTransformPointCloudSinCos(PointTypePose *tIn) ;
  
  /**
   * Transform point cloud using the latest frame tranform
   **/ 
  pcl::PointCloud<PointType>::Ptr transformPointCloud(
      pcl::PointCloud<PointType>::Ptr cloudIn);
  /**
   * Self defined point cloud transformation function 
   * as pcl one did not give good results
   **/ 
  pcl::PointCloud<PointType>::Ptr transformPointCloud(
      pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);
  /**
   * Publish TransformAftMapped to /aft_mapped
   **/ 
  void publishTF();
  /**
   * Publish laserCloudSurfFromMapDS
   **/ 
  void publishKeyPosesAndFrames();
  /**
   * Publish Surf, Corner, Outlier point clouds
   * of the entire map as one.
   **/ 
  void publishGlobalMap();

  // bool detectLoopClosure();
  // void performLoopClosure();
  
  /**
   * Extract nearest 50 frames as surroundings from 
   * the mapped environment
   * */
  void extractSurroundingKeyFrames();
  /**
   * Downsample current scan using downsampling filters
   **/
  void downsampleCurrentScan();
  /**
   * Using corner points generate coeffecients 
   * requred during LM Optimisation using laserCloudCornerLastDS
   **/
  void cornerOptimization(int iterCount);
  /**
   * Using surface points generate coeffecients 
   * requred during LM Optimisation using laserCloudSurfLastDS
   **/
  void surfOptimization(int iterCount);
  
  /**
   * Perform Levenberg-Marquardt optimisation technique
   * and update TransformToBeMapped. 
   * Return true if error acceptable.
   **/
  bool LMOptimization(int iterCount);
  /**
   * Use LM Optimisation and update the transform
   **/
  void scan2MapOptimization();
  /**
   * Update TransformAftMapped using latest estimate
   **/
  void saveKeyFramesAndFactor();
  /**
   * Correct poses if loop closure detected
   * Not used here
   **/ 
  void correctPoses();

  void clearCloud();

  /**
   * Function to get Corner Key Frame corresponding 
   * to particular index from the local map
   **/ 
  pcl::PointCloud<PointType>::Ptr getCornerCloudKeyFrame(int index);
  /**
   * Function to get Surf Key Frame corresponding 
   * to particular index from the local map
   **/
  pcl::PointCloud<PointType>::Ptr getSurfCloudKeyFrame(int index);
  /**
   * Function to get Outlier Key Frame corresponding 
   * to particular index from the local map
   **/
  pcl::PointCloud<PointType>::Ptr getOutlierCloudKeyFrame(int index);
};

#endif // MAPOPTIMIZATION_H
