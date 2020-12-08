#ifndef TRANSFORMFUSION_H
#define TRANSFORMFUSION_H

#include "lego_loam/utility.h"

class TransformFusion {
 private:
  ros::NodeHandle& nh;
  ros::Publisher pubLaserOdometry2; // Publish Odometry to /integrated_to_init
  ros::Subscriber subLaserOdometry;  // Subscribe to laser Odometery 
                                      // (/laser_odom_to_init)
  ros::Subscriber subOdomAftMapped;  // Subscribe to Mapped Odometry 
                                     // /aft_mapped_to_init  

  nav_msgs::Odometry laserOdometry2;
  tf::StampedTransform laserOdometryTrans2;
  tf::TransformBroadcaster tfBroadcaster2;

  tf::StampedTransform map_2_camera_init_Trans;
  tf::TransformBroadcaster tfBroadcasterMap2CameraInit;

  tf::StampedTransform camera_2_base_link_Trans;
  tf::TransformBroadcaster tfBroadcasterCamera2Baselink;

  float transformSum[6];   // The laser odometry data
  float transformIncre[6];
  float transformMapped[6];  // The Transform Associated with map
  float transformBefMapped[6];  
  float transformAftMapped[6];  // The Mapped Odometry

  std_msgs::Header currentHeader;

 public:
  TransformFusion(ros::NodeHandle& node);
  /**
   * Calculate TransformMapped wrt map using 
   * TransformBefMapped, TransformSum and TransformAftMapped
  */
  void transformAssociateToMap();
  /**
   * Handler for Laser Odometry that publishes TransformMapped and 
   * also the tf between /camera_init and /camera
  */
  void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry);
  /**
   * Handler for Mapped Odometry that updates values of 
   * TransformAftMapped and TransformBefMapped 
  */
  void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped);
};




#endif // TRANSFORMFUSION_H
