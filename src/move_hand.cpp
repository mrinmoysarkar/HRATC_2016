//author Mrinmoy sarkar
//email: sarkar.mrinmoy.bd@ieee.org

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <iostream>
#include <math.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <metal_detector_msgs/Coil.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <cstring>


using namespace std;



int main(int argc, char **argv)
{
        double min=-0.9;
        double max=0.9;
        double height=0.00;
        double speed=0.1;
        double acceleration=0.1;
            


    ros::init(argc, argv, "move_hand_node");
    ros::NodeHandle n;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_("/arm_controller/follow_joint_trajectory", true);
    ROS_INFO("Sweep -- Waiting for the action server to start...");
    ac_.waitForServer();
    ROS_INFO("Sweep -- Got it!");
    //metal_detector md(&n,&ac);
    double position = min;

while(ros::ok())
    {
       
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.header.stamp = ros::Time::now();
        goal.trajectory.joint_names.resize(2);
        goal.trajectory.points.resize(1);
        goal.trajectory.joint_names[0] = "upper_arm_joint";
        goal.trajectory.points[0].positions.push_back(height);
        goal.trajectory.points[0].velocities.push_back(speed);
        goal.trajectory.points[0].accelerations.push_back(acceleration);
        goal.trajectory.joint_names[1] = "arm_axel_joint";
        goal.trajectory.points[0].positions.push_back(position = position == min ? max : min);
        goal.trajectory.points[0].velocities.push_back(speed);
        goal.trajectory.points[0].accelerations.push_back(acceleration);
        goal.trajectory.points[0].time_from_start = ros::Duration(2.7);
        goal.goal_tolerance.resize(4);
        goal.goal_tolerance[0].name = "upper_arm_joint";
        goal.goal_tolerance[0].position = 0.01;
        goal.goal_tolerance[1].name = "arm_axel_joint";
        goal.goal_tolerance[1].position = 0.01;
        goal.goal_time_tolerance = ros::Duration(0.5);

        ac_.sendGoal(goal);

        // Wait for the action to return
        bool finished_before_timeout = ac_.waitForResult(ros::Duration(30.0));

        if(!finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac_.getState();
            ROS_ERROR("Sweep -- %s!", state.toString().c_str());
        }
        //cout << "MOVING" << endl;
    }





    ros::spin();
    return 0;
}
