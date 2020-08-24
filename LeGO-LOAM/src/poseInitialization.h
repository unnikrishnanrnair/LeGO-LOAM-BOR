#ifndef POSEINITIALIZATION_H
#define POSEINITIALIZATION_H

#include "lego_loam/utility.h"

class PoseInitialization {

public:
	PoseInitialization( ros::NodeHandle& node);

	~PoseInitialization();

private:
	ros::NodeHandle& nh;

	ros::Publisher pubInitialPose;

	std::thread _run_thread;

	double init_pose_x;
	double init_pose_y;
	double init_pose_z;
	double init_pose_r;
	double init_pose_p;
	double init_pose_y;

	std::string init_pose_file;

	nav_msgs::Odometry intiPose;

private:
	void readPoseFromFile();
	void publishIntialPose();
	void runPoseInitialization();

};

#endif