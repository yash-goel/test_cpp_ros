#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/default_topics.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>
#include <iostream>
#include "mav_msgs/common.h"

class my_way_point
{
    float x;
    float y;
    float z;

    my_way_point()
    {
        x = 0;
        y = 0;
        z = 0;
    }    
};
    

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_node");
    ros::NodeHandle n;

    int i = 0;

    // ros::Subscriber way_sub = n.subscribe<geometry_msgs::PoseArray>("/waypoints", 10, waycb);

    ros::Publisher traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/hummingbird/command/trajectory", 20);
    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>("trajectory_traject", 10);

    ros::Rate sleep_rate(100);

    // for starting the loop once
    int traj_size = 10;

    while (ros::ok())
    {   
         
        if (i > traj_size - 1)
            i = 0;

        mav_trajectory_generation::Vertex::Vector vertices;
        const int dimension = 3;
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;
        mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

        //enter way point and size
        int way_size = 9;

        // Time count
        ros::Time t0 = ros::Time::now();

        start.makeStartOrEnd(Eigen::Vector3d(0,0,0), derivative_to_optimize);
        vertices.push_back(start);
       
        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0, 5, 4));
        vertices.push_back(middle);        

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(8,5,7));
        vertices.push_back(middle);        

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(6,4,3));
        // vertices.push_back(middle);        

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(4,-3,3));
        // vertices.push_back(middle);        

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(2,2,3));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0,-1,3));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1, 3, 2));
        // vertices.push_back(middle);        

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(2,-5,4));
        // vertices.push_back(middle);        

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(3,7,6));
        // vertices.push_back(middle);        

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(4,-9,8));
        // vertices.push_back(middle);        

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(5,11,10));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-5,11,10));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0, 11,0));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(5,-3,4));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-5,-3,4));
        // vertices.push_back(middle);

        end.makeStartOrEnd(Eigen::Vector3d(4,3,0), derivative_to_optimize);
        vertices.push_back(end);

        //compute the segment times
        std::vector<double> segment_times;
        const double v_max = 1.5;
        const double a_max = 3.0;
        const double magic_fabian_constant = 6.5; // A tuning parameter
        segment_times = estimateSegmentTimes(vertices, v_max, a_max, magic_fabian_constant);


        //N denotes the number of coefficients of the underlying polynomial
        //N has to be even.
        //If we want the trajectories to be snap-continuous, N needs to be at least 10.

        const int N = 10;
        mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
        opt.solveLinear();

        //ROS_INFO("Take %f sec to get optimal traject", (ros::Time::now() - t0).toSec());

        //Obtain the polynomial segments
        mav_trajectory_generation::Segment::Vector segments;
        opt.getSegments(&segments);

            //creating Trajectories
            
        mav_trajectory_generation::Trajectory trajectory;
        opt.getTrajectory(&trajectory);
            
        //evaluating the trajectory at particular instances of time
        // Single sample:
        double sampling_time = 2.0;
        int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
        Eigen::VectorXd sample = trajectory.evaluate(sampling_time, derivative_order);

        // Sample range:
        double t_start = 2.0;
        double t_end = 20.0;    // default = 10.0
        double dt = 0.01;      // default = 0.01
        std::vector<Eigen::VectorXd> result;
        std::vector<double> sampling_times; // Optional.
        trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);
        
        // sampled trajectory point

        static constexpr double kDefaultSamplingTime = 0.001;   //default = 0.1
        geometry_msgs::Pose traj_point;
        mav_msgs::EigenTrajectoryPoint::Vector flat_states;
        bool suces = mav_trajectory_generation::sampleWholeTrajectory(trajectory, kDefaultSamplingTime, &flat_states);
        
        traj_size = flat_states.size();        

        // conversion to pose stamped for sending to mpc controller to take control actions and sending the pose in loop
        
        trajectory_msgs::MultiDOFJointTrajectory traj_msg;
        mav_msgs::msgMultiDofJointTrajectoryFromEigen(flat_states[i], &traj_msg);

        // publishing trajectory to send to non linear mpc controller

        traj_pub.publish(traj_msg);
    
        //visualizing Trajectories
        visualization_msgs::MarkerArray markers;
        double distance = 1.6; // Distance by which to seperate additional markers. Set 0.0 to disable.
        std::string frame_id = "world";
        // From Trajectory class:
        mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);

        vis_pub.publish(markers);

        i++;
        
        ros::spinOnce();
        vertices.clear();
        sleep_rate.sleep();
    }
    return 0;
}
