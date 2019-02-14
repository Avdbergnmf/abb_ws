// This package takes care of the communication between the geomagic_touch_m package (specifically the pose state topic)
// and the abb_driver package. It needs to take the output of the omni and translate it to a relative goal for the abb
// robot.

#include "pathmaker_abb/pathmaker_abb.h"

void commClass::init()
{
    // init subscribers
    abb_joint_state_sub = nh.subscribe("joint_states", 10, &commClass::ABB_state_callback, this);
    geo_butt_sub = nh.subscribe("geo_buttons_m", 10, &commClass::GeoMagic_butt_callback, this);

    // init publishers
    joint_traj_act_goal_pub = nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/joint_trajectory_action/goal", 1);
    // joint_traj_act_goal_pub = nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("joint_path_command", 10);
    // init variables 
    rate = 5;
    link_arm = 0;
    super_link_arm = 0;
    toggle_helper = 0;
    do_publish = 0;

    // Settings
    k = 10;

    zero_pose.position.x = 0;zero_pose.position.y = 0;zero_pose.position.z = 0;
    zero_pose.orientation.x = 0;zero_pose.orientation.y = 0;zero_pose.orientation.z = 0;zero_pose.orientation.w = 0;

    last_abb_pose = zero_pose;

    // ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description"); //"abb_irb1200_5_90"); will find name automatically if abb_.._support node is running
    kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state_temp(new robot_state::RobotState(kinematic_model)); // need a delete statement somewhere
    kinematic_state = kinematic_state_temp;

    // kinematic_state->setToDefaultValues(); // dont rlly wanna do this...
    robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    
    newGoal.goal.trajectory.joint_names.resize(6);
    newGoal.goal.trajectory.joint_names = joint_names;
}

void commClass::ABB_state_callback(const sensor_msgs::JointState &abb__joint_state) 
{
}

void commClass::GeoMagic_butt_callback(const pathmaker_abb::GeomagicButtonEvent &buttons_state)
{
    // toggle link arm
    if (buttons_state.white_button){ 
        if (!toggle_helper) {
            toggle_helper = 1; // so it doesn't instantly toggle on/off but requires you to let go first
            if (super_link_arm) {
                link_arm = 0;
                super_link_arm = 0;
            }
            else {
                super_link_arm = 1;
            }
        }
    }
    else {
        toggle_helper = 0;
    }
    // toggle by holding
    if (super_link_arm) {
        if (buttons_state.grey_button) { link_arm = 0; }
        else { link_arm = 1; }
    }
}



void commClass::publish_goal()
{
    // Find geo state (stylus)
    tf::TransformListener tf_listener;
    tf_listener.waitForTransform("base", "stylus", ros::Time(0), ros::Duration(0.25));
    tf_listener.lookupTransform("base", "stylus", ros::Time(0), TF_geo_pose);
    
    // Do a rotation to match the real scene
    tf::Quaternion frameRotation = TF_geo_pose.getRotation();
    TF_geo_pose.setRotation( tf::Quaternion(0,0,0.707,0.707)*frameRotation );  // 90 deg around z axis: from https://quaternions.online/

    // Transform the translation aswel and put into geometry msg
    geo_pose.position.x = - TF_geo_pose.getOrigin().getY();
    geo_pose.position.y =   TF_geo_pose.getOrigin().getX();
    geo_pose.position.z =   TF_geo_pose.getOrigin().getZ();
    geo_pose.orientation.x = TF_geo_pose.getRotation().getX();
    geo_pose.orientation.y = TF_geo_pose.getRotation().getY();
    geo_pose.orientation.z = TF_geo_pose.getRotation().getZ();
    geo_pose.orientation.w = TF_geo_pose.getRotation().getW();

    // Find ABB state http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/robot_model_and_robot_state/robot_model_and_robot_state_tutorial.html
    robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator"); // just get it this way....
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values); // This is a pointer object!
    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("link_6");
    tf::poseEigenToMsg(end_effector_state, abb_pose); //put in Geometry pose msg

    if (link_arm) {
        // >>> Geo Stuff
        // calculate relative pose geo
        relative_geo_pose.position.x = geo_pose.position.x - start_geo_pose.position.x; 
        relative_geo_pose.position.y = geo_pose.position.y - start_geo_pose.position.y;
        relative_geo_pose.position.z = geo_pose.position.z - start_geo_pose.position.z;

        tf::Quaternion q2( geo_pose.orientation.x, geo_pose.orientation.y, geo_pose.orientation.z, geo_pose.orientation.w);
        tf::Quaternion q1_inv( start_geo_pose.orientation.x, start_geo_pose.orientation.y, start_geo_pose.orientation.z, -start_geo_pose.orientation.w);
        tf::Quaternion qr = q2 * q1_inv; // q_r (from q1 to q2) = q_2*q_1_inv;
        qr.normalize();
        tf::quaternionTFToMsg(qr, relative_geo_pose.orientation); // put in geometry msg
        
        // >>> ABB stuff
        // Convert to relative abb pose
        target_abb_pose.position.x = start_abb_pose.position.x + k*relative_geo_pose.position.x; 
        target_abb_pose.position.y = start_abb_pose.position.y + k*relative_geo_pose.position.y;
        target_abb_pose.position.z = start_abb_pose.position.z + k*relative_geo_pose.position.z;

        tf::Quaternion q_abb(start_abb_pose.orientation.x, start_abb_pose.orientation.y, start_abb_pose.orientation.z, start_abb_pose.orientation.w);
        tf::Quaternion qr_abb = qr*q_abb;
        qr_abb.normalize();
        tf::quaternionTFToMsg(qr_abb, target_abb_pose.orientation); // put in geometry msg

        // convert it to eigen 
        tf::poseMsgToEigen(target_abb_pose, target_end_effector_state);

        // send to Rviz
        tf::Transform broadCastTF;
        broadCastTF.setRotation(qr_abb);
        broadCastTF.setOrigin(tf::Vector3(target_abb_pose.position.x,target_abb_pose.position.y,target_abb_pose.position.z));
        tf::StampedTransform broadCastTFStamped(broadCastTF, ros::Time::now(), "base_link", "target_frame");

        brtest.sendTransform( broadCastTFStamped );
    }
    else {
        // Geo Stuff
        relative_geo_pose = zero_pose;

        start_geo_pose = geo_pose; // set the current geo state as the reference point

        // ABB stuff
        target_abb_pose = abb_pose; 
        start_abb_pose = abb_pose; // set the curr abb state as the reference point

    }

    // check the difference in position & orientation
    geometry_msgs::Pose diffPose;
    diffPose.position.x = (target_abb_pose.position.x - last_abb_pose.position.x) * (target_abb_pose.position.x - last_abb_pose.position.x);
    diffPose.position.y = (target_abb_pose.position.y - last_abb_pose.position.y) * (target_abb_pose.position.y - last_abb_pose.position.y);
    diffPose.position.z = (target_abb_pose.position.z - last_abb_pose.position.z) * (target_abb_pose.position.z - last_abb_pose.position.z);
    lengthDiff = sqrt(diffPose.position.x+diffPose.position.y+diffPose.position.z);

    diffPose.orientation.x = (target_abb_pose.orientation.x - last_abb_pose.orientation.x) * (target_abb_pose.orientation.x - last_abb_pose.orientation.x);
    diffPose.orientation.y = (target_abb_pose.orientation.y - last_abb_pose.orientation.y) * (target_abb_pose.orientation.y - last_abb_pose.orientation.y);
    diffPose.orientation.z = (target_abb_pose.orientation.z - last_abb_pose.orientation.z) * (target_abb_pose.orientation.z - last_abb_pose.orientation.z);
    diffPose.orientation.w = (target_abb_pose.orientation.w - last_abb_pose.orientation.w) * (target_abb_pose.orientation.w - last_abb_pose.orientation.w);
    oriDiff = sqrt(diffPose.orientation.x+diffPose.orientation.y+diffPose.orientation.z+diffPose.orientation.w);

    last_abb_pose = target_abb_pose; // set the last pose for reference next loop


    // >>> Actually do the publishing stuff <<<

    // ROS_INFO("Link arm state: " << link_arm); // is the arm linked atm?
    vector<double>().swap(goalAngels_vec); // clear the joint angles vector

    newGoal.header.stamp = ros::Time::now();
    newGoal.goal.trajectory.header.stamp = ros::Time::now(); //dunno welke ik van deze persee moet hebben...?

    // ROS_INFO_STREAM("lengthDiff: " << lengthDiff);
    // ROS_INFO_STREAM("oriDiff: " << oriDiff);

    if (lengthDiff > 0.01 || oriDiff>0.01) { // only publish if there is enough of a difference

        // Inverse kinematics
        std::size_t attempts = 10; // how many attempts to solve
        double timeout = 0.01; // timeout per attempt
        bool found_ik = kinematic_state->setFromIK(joint_model_group, target_end_effector_state, attempts, timeout);

        if (found_ik) {
            kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
            do_publish = 1;
            for (int i = 0; i < 6; i++) {
                goalAngels_vec.push_back(joint_values[i]);
            }

            newGoal.goal.trajectory.points.resize(1);
            newGoal.goal.trajectory.points[0].positions.resize(6);
            
            newGoal.goal.trajectory.points[0].positions = goalAngels_vec;
        } 
        else {
            ROS_INFO("Did not find IK solution");
            do_publish = 0;
        }
        // do_publish=1;
        if (do_publish){
            joint_traj_act_goal_pub.publish(newGoal);
            ROS_INFO_STREAM("Publishing new goal");
            // for (int i=0;i<6;i++){
            //     ROS_INFO_STREAM("Target joint angle " << i << " : " << goalAngels_vec[i]);
            // }
        }    
    }
    else { 
        ROS_INFO_STREAM("Not enough difference, not publishing");
    }
}


int main(int argc, char **argv)
{
    // Initialisation
    ros::init(argc, argv, "pathmaker_abb");
    sleep(2);
    
    commClass commObj;

    commObj.init();

    // ROS loop
    ROS_INFO("Starting communication");
    ros::Rate loop_rate(commObj.rate);

    while (ros::ok())
    {
        commObj.publish_goal();

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Ended communication loop");
    return 0;
}
