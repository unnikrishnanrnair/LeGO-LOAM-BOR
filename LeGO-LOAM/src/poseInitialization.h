#ifndef POSEINITIALIZATION_H
#define POSEINITIALIZATION_H

#include "lego_loam/utility.h"
#include <ros/package.h>

/**
 * PoseInitialization class publishes the initial position of vehicle
 */
class PoseInitialization {

public:
	/**
	 * Default Constructor
	 */
	PoseInitialization( ros::NodeHandle& node);

	~PoseInitialization();

	/**
	 * Main Function
	 */
	void runPoseInitialization();

private:
	ros::NodeHandle& nh;

	ros::Publisher pubInitialPose;	// vehcile initial position publisher

	std::thread _run_thread;

	double init_pose_x;		// vehicle initial position x coordinate
	double init_pose_y;		// vehicle initial position y coordinate
	double init_pose_z;		// vehicle initial position z coordinate
	double init_pose_roll;	// vehicle initial roll
	double init_pose_pitch;	// vehicle initial pitch
	double init_pose_yaw;	// vehicle initial yaw

	std::string init_pose_file;	// location of inital position file

	nav_msgs::Odometry initPose; // initial position as odometry message

private:
	/**
	 * Read the initial position of vehicle from init_pose_file
	 * 
	 * FILE FORMAT:
	 * x y z roll pitch yaw (6 space separated values in this order)
	 *
	 */
	void readPoseFromFile();

	/**
	 * publishes the vehicle's initial position as odometry message on '/tf/odometry'
	 */
	void publishInitialPose();

};

#endif