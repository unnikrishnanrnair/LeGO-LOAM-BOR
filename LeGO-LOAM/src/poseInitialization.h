#ifndef POSEINITIALIZATION_H
#define POSEINITIALIZATION_H

#include "lego_loam/utility.h"
#include <ros/package.h>

class PoseInitialization {

public:
	PoseInitialization( ros::NodeHandle& node);

	~PoseInitialization();

	void runPoseInitialization();
private:
	ros::NodeHandle& nh;

	ros::Publisher pubInitialPose;

	std::thread _run_thread;

	double init_pose_x;
	double init_pose_y;
	double init_pose_z;
	double init_pose_raw;
	double init_pose_pitch;
	double init_pose_yaw;

	std::string init_pose_file;

	nav_msgs::Odometry intiPose;

private:
	void readPoseFromFile();
	void publishIntialPose();

};

#endif