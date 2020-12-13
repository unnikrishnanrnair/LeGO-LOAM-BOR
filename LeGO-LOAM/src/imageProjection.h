#ifndef IMAGEPROJECTION_H
#define IMAGEPROJECTION_H

#include "lego_loam/utility.h"
#include "lego_loam/channel.h"
#include <Eigen/QR>

/**
 * Image Projection Class
 * 
 * For incoming laser scan
 *  Finds the ground points
 *  Removes the ground points and outlier points
 */
class ImageProjection {
 public:

  /**
   * Default Constructor
   */
  ImageProjection(ros::NodeHandle& nh,
                  Channel<ProjectionOut>& output_channel);

  ~ImageProjection() = default;

  /**
   * Handles the incoming laser cloud scan
   */
  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);

 private:
  /**
   * Finds the start and end orientation of the point cloud.
   */
  void findStartEndAngle();

  /**
   * Clears all the parameters
   */
  void resetParameters();

  /**
   * Ranges the point cloud
   *
   * fills the _range_mat
   * calculates the row and column index for each point and stores its range in _range_mat(row_index, column_index)
   */
  void projectPointCloud();

  /**
   * Marks the ground points
   * 
   * fills the _ground_mat
   * 
   * _ground_mat
   *  -1, no valid info to check if ground of not
   *   0, initial value, after validation, means not ground
   *   1, ground
   */
  void groundRemoval();

  /**
   * segments the point cloud
   * 
   * label the points (if unlabelled) using labelComponents() function
   * outlier points are removed (with label value 999999)
   * majority of ground points are removed (with label value -1)
   */
  void cloudSegmentation();

  /**
   * label the points
   *
   * fills the _label_mat
   * valid points has label value > 0
   * outlier points has label value = 999999
   * ground points has label value = -1
   * all the points of the same segment has the same label value
   */
  void labelComponents(int row, int col);
  
  /**
   * publishes:
   *  _outlier_cloud
   *  _segmented_cloud
   *  _full_cloud
   *  _ground_cloud
   *  _segmented_cloud_pure
   *  _full_info_cloud
   */
  void publishClouds();

  pcl::PointCloud<PointType2>::Ptr _laser_cloud_in; // stores current laser scan

  pcl::PointCloud<PointType>::Ptr _full_cloud; // stores current full cloud
  pcl::PointCloud<PointType>::Ptr _full_info_cloud; // stores current full cloud + range

  pcl::PointCloud<PointType>::Ptr _ground_cloud; // stores ground points
  pcl::PointCloud<PointType>::Ptr _segmented_cloud; // stores segmented cloud
  pcl::PointCloud<PointType>::Ptr _segmented_cloud_pure; // stores segmented cloud for visualization
  pcl::PointCloud<PointType>::Ptr _outlier_cloud; // stores outlier points

  ros::NodeHandle& _nh;
  int _vertical_scans;
  int _horizontal_scans;
  float _ang_bottom;
  float _ang_resolution_X;
  float _ang_resolution_Y;
  float _segment_alpha_X;
  float _segment_alpha_Y;
  float _segment_theta;
  int _segment_valid_point_num;
  int _segment_valid_line_num;
  int _ground_scan_index;
  float _sensor_mount_angle;

  Channel<ProjectionOut>& _output_channel;

  ros::Subscriber _sub_laser_cloud;

  ros::Publisher _pub_full_cloud;
  ros::Publisher _pub_full_info_cloud;

  ros::Publisher _pub_ground_cloud;
  ros::Publisher _pub_segmented_cloud;
  ros::Publisher _pub_segmented_cloud_pure;
  ros::Publisher _pub_segmented_cloud_info;
  ros::Publisher _pub_outlier_cloud;

  cloud_msgs::cloud_info _seg_msg;

  int _label_count;

  Eigen::MatrixXf _range_mat;   // range matrix for range image
  Eigen::MatrixXi _label_mat;   // label matrix for segmentaiton marking
  Eigen::Matrix<int8_t,Eigen::Dynamic,Eigen::Dynamic> _ground_mat;  // ground matrix for ground cloud marking


};



#endif  // IMAGEPROJECTION_H
