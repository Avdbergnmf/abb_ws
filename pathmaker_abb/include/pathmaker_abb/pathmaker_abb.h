// This contains the header file defining the required classes etc.

#ifndef PATHMAKER_ABB_H
#define PATHMAKER_ABB_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <math.h>
#include <iostream>
#include <string.h>

#include <geometry_msgs/Quaternion.h> // do i need this? 
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include <pathmaker_abb/GeomagicButtonEvent.h> // custom message van de geomagic touch (door Guido)
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

// kinematics stuff
#include <urdf/model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_broadcaster.h>
// #include <transform_listener.h>


// General Declarations
using namespace std;

// 

class commClass {
    public:
        ros::NodeHandle nh;
        ros::Publisher joint_traj_act_goal_pub;
        ros::Subscriber geo_state_sub, geo_butt_sub, abb_joint_state_sub;

        int rate;
        bool super_link_arm, link_arm, toggle_helper, do_publish;
        float k; // tranlation gain
        double goalAngles[6];

        sensor_msgs::JointState abb__joint_state;
        geometry_msgs::Pose relative_geo_pose, start_geo_pose, start_geo_pose_inv, zero_pose;
        geometry_msgs::Pose target_abb_pose, start_abb_pose, start_abb_pose_inv, abb_pose;

        tf::StampedTransform TF_geo_pose;
        geometry_msgs::Pose geo_pose, last_abb_pose;
        geometry_msgs::PoseStamped geo_sPose;

        pathmaker_abb::GeomagicButtonEvent buttons_state;

        control_msgs::FollowJointTrajectoryActionGoal newGoal;

        vector<double> abb_angles_vec, goalAngels_vec;
        vector<string> joint_name_vec;
        
        // kinematics stuff
        robot_model::RobotModelPtr kinematic_model;
        robot_state::RobotStatePtr kinematic_state;
        vector<double> joint_values, oldAngles;
        double lengthDiff, oriDiff;

        tf::TransformBroadcaster brtest;
        Eigen::Affine3d end_effector_state;
        Eigen::Affine3d target_end_effector_state;

        void init();
        void ABB_state_callback(const sensor_msgs::JointState &abb__joint_state);
        void GeoMagic_butt_callback(const pathmaker_abb::GeomagicButtonEvent &buttons_state);

        void publish_goal();

};





#endif // PATHMAKER_ABB_H



