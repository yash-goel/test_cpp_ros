#include <ros/ros.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>
#include <iostream>
#include <tf/tfMessage.h>
#include <cmath>
#include <rosbag/bag.h>
#include "mav_msgs/common.h"

// normalising angle between 0 to 2*pi
const double pi = 3.141592653589793238463;

double NormaliseAngle(double x)
{
    x = fmod(x,2*pi);
    if (x < 0)
        x += 2*pi;
    return x;
}


int main(int argc, char **argv)
{	
	long int i = 0;
	ros::init(argc, argv, "nonlinear_trajectory_node");

	float radius = 3;

	ros:: NodeHandle traj_node;
	ros:: Publisher trajectory_pub = traj_node.advertise<geometry_msgs::PoseStamped>("/hummingbird/command/pose", 10);
	ros::Rate rate(100);

	// parameters for lemniscate

	int p = 9;
	int q = 0;

	// parameters for following line passing through origin ax + by + cz = 0

	float a = 4;
	float b = -7;
	float c = -1;

	float rise = 0;
	float rise_delta = 0.001;

	// initialising for square trajectory
	float sq_half_diag = 5;
	float sq_x = 5;
	float sq_y = 0;
	int square_flag = 0;

	while(ros::ok())
	{	
		float pnt = 0.001*i;

		// trajectory_msgs::MultiDOFJointTrajectory reference_point;
		geometry_msgs::PoseStamped reference_point;

		// parametric variables

		float theta = pnt;
		float r = sqrt(2*a*a*(cos(2*theta + b)));

		float norm_theta = NormaliseAngle(theta);

		// for square

		if(sq_x>=sq_half_diag)
		{
			square_flag = 1;
		}
		else if(sq_y<=-sq_half_diag)
		{
			square_flag = 2;
		}
		else if(sq_x<=-sq_half_diag)
		{
			square_flag = 3;
		}
		else if(sq_y>=sq_half_diag)
		{
			square_flag = 4;
		}

		if(square_flag==1)
		{
			sq_x -= 0.001;
			sq_y = -sq_half_diag + sq_x;
		}
		else if(square_flag==2)
		{
			sq_y += 0.001;
			sq_x = -sq_half_diag - sq_y;
		}
		else if(square_flag==3)
		{
			sq_x += 0.001;
			sq_y = sq_half_diag + sq_x;
		}
		else if(square_flag==4)
		{
			sq_y -= 0.001;
			sq_x = sq_half_diag - sq_y;
		}
		

		if((rise>5) & (rise_delta > 0))
		{
			rise_delta = -0.001;
		}
		else if((rise<0.001) & (rise_delta < 0))
		{
			rise_delta = 0.001;
		}
		else
		{
			rise_delta = rise_delta;
		}

		rise = rise + rise_delta;

		// solving for line

		float line_x = -pnt;
		float line_y = -pnt;
		float line_z = -((a*line_x + b*line_y)/c); 

		reference_point.pose.position.x = 0;
		reference_point.pose.position.y = 0;
		reference_point.pose.position.z = rise;
		
		reference_point.pose.orientation.x = 0;
		reference_point.pose.orientation.y = 0;
		reference_point.pose.orientation.z = 0;
		reference_point.pose.orientation.w = 0;

		ros::Time update_time = ros::Time::now();
  		reference_point.header.stamp = update_time;
  		reference_point.header.frame_id = "rotors_joy_frame";

        // mav_msgs::EigenTrajectoryPoint reference_traj;
        // mav_msgs::eigenTrajectoryPointFromPoseMsg(reference_point, &reference_traj);
        
        trajectory_pub.publish(reference_point);

        i++;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

