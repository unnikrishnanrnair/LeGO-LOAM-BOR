#include "poseInitialization.h"

PoseInitialization::PoseInitialization(ros::NodeHandle &node) : nh(node) {

	init_pose_file = ros::package::getPath("lego_loam_bor") + "/initalRobotPose.txt";

	init_pose_x = 0;
	init_pose_y = 0;
	init_pose_z = 0;
	init_pose_roll = 0;
	init_pose_pitch = 0;
	init_pose_yaw = 0;

	pubInitialPose = nh.advertise<nav_msgs::Odometry>("/tf/odometry", 5);

	_run_thread = std::thread (&PoseInitialization::runPoseInitialization, this);
}

PoseInitialization::~PoseInitialization(){
	_run_thread.join();
}

void PoseInitialization::readPoseFromFile(){

	try{
		std::ifstream file(init_pose_file);

		if(!(file >> init_pose_x >> init_pose_y >> init_pose_z >> init_pose_roll >> init_pose_pitch >> init_pose_yaw)){
			std::cout << "Can't read init_pose_file" << std::endl;
		}

		std::cout << init_pose_x << std::endl;
		std::cout << init_pose_y << std::endl;
		std::cout << init_pose_z << std::endl;
		std::cout << init_pose_roll << std::endl;
		std::cout << init_pose_pitch << std::endl;
		std::cout << init_pose_yaw << std::endl;
	}
	catch(const std::exception& e){
		std::cerr << e.what() << std::endl;
	}

	geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(init_pose_roll, init_pose_pitch, init_pose_yaw);

	intiPose.pose.pose.orientation.x = geoQuat.x;
  	intiPose.pose.pose.orientation.y = geoQuat.y;
  	intiPose.pose.pose.orientation.z = geoQuat.z;
  	intiPose.pose.pose.orientation.w = geoQuat.w;
  	intiPose.pose.pose.position.x = init_pose_x;
  	intiPose.pose.pose.position.y = init_pose_y;
  	intiPose.pose.pose.position.z = init_pose_z;
}

void PoseInitialization::publishInitialPose(){

	if(pubInitialPose.getNumSubscribers() != 0){
  		pubInitialPose.publish(intiPose);
	}
}

void PoseInitialization::runPoseInitialization(){

	readPoseFromFile();

	while(ros::ok()){
		
		if(!ros::ok())
			break;

		publishInitialPose();
	}	
}

