#ifndef FEATUREASSOCIATION_H
#define FEATUREASSOCIATION_H

#include "lego_loam/utility.h"
#include "lego_loam/channel.h"
#include "lego_loam/nanoflann_pcl.h"
#include <Eigen/Eigenvalues>
#include <Eigen/QR>

/**
 * Feature Association
 * 
 * Extracts the surface and edge features for the laser scan
 * Takes input from imageProjection
 * Also computes and broadcasts the laser odometry
 */
class FeatureAssociation {

 public:
  
  /**
   * Default Constructor
   */
  FeatureAssociation( ros::NodeHandle& node,
                     Channel<ProjectionOut>& input_channel,
                     Channel<AssociationOut>& output_channel);

  ~FeatureAssociation();

  /**
   * Main Function
   */
  void runFeatureAssociation();

 private:

  ros::NodeHandle& nh;

  int _vertical_scans;
  int _horizontal_scans;
  float _scan_period;
  float _edge_threshold;
  float _surf_threshold;
  float _nearest_feature_dist_sqr;
  int _mapping_frequency_div;

  std::thread _run_thread;

  Channel<ProjectionOut>& _input_channel;
  Channel<AssociationOut>& _output_channel;

  ros::Publisher pubCornerPointsSharp;
  ros::Publisher pubCornerPointsLessSharp;
  ros::Publisher pubSurfPointsFlat;
  ros::Publisher pubSurfPointsLessFlat;

  pcl::PointCloud<PointType>::Ptr segmentedCloud; // stores the segmented cloud
  pcl::PointCloud<PointType>::Ptr outlierCloud; // stores the outlier cloud

  pcl::PointCloud<PointType>::Ptr cornerPointsSharp; // stores sharp corner features
  pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp; // stores less sharp corner features
  pcl::PointCloud<PointType>::Ptr surfPointsFlat; // stores flat surface features
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlat; // stores less flat surface features

  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;

  pcl::VoxelGrid<PointType> downSizeFilter;

  double timeScanCur;

  cloud_msgs::cloud_info segInfo;
  std_msgs::Header cloudHeader;

  int systemInitCount;
  bool systemInited;

  std::vector<smoothness_t> cloudSmoothness;
  std::vector<float> cloudCurvature;
  std::vector<int> cloudNeighborPicked;
  std::vector<int> cloudLabel;

  ros::Publisher _pub_cloud_corner_last;
  ros::Publisher _pub_cloud_surf_last;
  ros::Publisher pubLaserOdometry;
  ros::Publisher _pub_outlier_cloudLast;

  int skipFrameNum;
  bool systemInitedLM;

  int laserCloudCornerLastNum;
  int laserCloudSurfLastNum;

  std::vector<int> pointSelCornerInd;
  std::vector<float> pointSearchCornerInd1;
  std::vector<float> pointSearchCornerInd2;

  std::vector<int> pointSelSurfInd;
  std::vector<float> pointSearchSurfInd1;
  std::vector<float> pointSearchSurfInd2;
  std::vector<float> pointSearchSurfInd3;

  float transformCur[6]; // stores the current transform of vehcile (considering starting point sa origin)
  float transformSum[6]; // stores the current transform after integration with initial position

  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;  // stores corner features of previous scan
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // stores surface features of previous scan
  pcl::PointCloud<PointType>::Ptr laserCloudOri; 
  pcl::PointCloud<PointType>::Ptr coeffSel;

  nanoflann::KdTreeFLANN<PointType> kdtreeCornerLast; // kd tree for finding corresponding corner features
  nanoflann::KdTreeFLANN<PointType> kdtreeSurfLast; // kd tree for finding corresponding surface features

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  nav_msgs::Odometry laserOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform laserOdometryTrans;

  bool isDegenerate;

  int frameCount;
  size_t _cycle_count;

 private:

  /**
   * initializes all the parameters
   */
  void initializationValue();
  
  /**
   * adjust the segmented cloud distortion
   *
   * DISTORTION:
   *  x is y
   *  y is z
   *  z is x
   */
  void adjustDistortion();
  
  /**
   * calculates the smoothness value for each point
   */
  void calculateSmoothness();
  
  /**
   * marks the occluded points in current scan
   */
  void markOccludedPoints();
  
  /**
   * extracts the surface and corner features 
   */
  void extractFeatures();

 

  /**
   * 
   */
  void TransformToStart(PointType const *const pi, PointType *const po);
  
  /**
   *
   */
  void TransformToEnd(PointType const *const pi, PointType *const po);


  
  /**
   * Finds the relative rotation between (cx, cy, cz) and (lx, ly, lz)
   */
  void AccumulateRotation(float cx, float cy, float cz, float lx, float ly,
                          float lz, float &ox, float &oy, float &oz);



  /**
   * finds the corner features corresponding to current scan corner features.
   */ 
  void findCorrespondingCornerFeatures(int iterCount);
  
  /**
   * finds the surf features corresponding to current scan corner features.
   */
  void findCorrespondingSurfFeatures(int iterCount);



  /**
   * calculates the relative tranform using current surface features and their corresponding surface features (from previous scans). 
   *
   * calculates roll, pitch, tz
   */
  bool calculateTransformationSurf(int iterCount);
  
  /**
   * calculates the relative tranform using current corner features and their corresponding corner features (from previous scans). 
   *
   * calculates tx, ty, yaw
   */
  bool calculateTransformationCorner(int iterCount);
  
  /**
   * calculates the relative tranform using current features and their corresponding features (from previous scans). 
   *
   * calculates roll, pitch, yaw, tx, ty and tz
   */
  bool calculateTransformation(int iterCount);

  

  /**
   * initializes parameters and kdTrees for finding corresponding feautures.
   */
  void checkSystemInitialization();
  
  /**
   * finds the relative transform with the previous scan and updates the current transform accordingly
   */
  void updateTransformation();



  /**
   * integrates the vehicles initial transformation with current tranformation.
   *
   * stores the integrated transform in transformSum
   */
  void integrateTransformation();
  
  /**
   * publishes cloud for visualization
   *
   * publishes:
   *  cornerPointsSharp
   *  cornerPointsLessSharp
   *  surfPointsFlat
   *  surfPointsLessFlat
   */
  void publishCloud();
  
  /**
   * Broadcasts the integrated transform stored in transformSum (laser odometry)
   */
  void publishOdometry();

  

  /**
   * adjusts the distortion in outlier cloud
   *
   * DISTORTION:
   *  x is y
   *  y is z
   *  z is x
   */
  void adjustOutlierCloud();
  
  /**
   * publishes cloud to mapOptimization
   *
   * publishes:
   *  outlierCloud
   *  laserCloudCornerLast
   *  laserCloudSurfLast
   */
  void publishCloudsLast();

};

#endif // FEATUREASSOCIATION_H
