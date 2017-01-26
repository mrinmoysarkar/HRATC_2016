/*******************************************************************
*
* Author: Mrinmoy Sarkar
* email: sarkar.mrinmoy.bd@ieee.org
*
*********************************************************************/

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

class metal_detector
{
    public:
        metal_detector(ros::NodeHandle *nh, actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *ac);    
    private:
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *ac_;
        ros::NodeHandle *nh_;  
        double min;
        double max;
        double height;
        double speed;
        double acceleration;
        double minmin;
        double maxmax;
        double del;
        int coil_number;
        bool can_move; 
        message_filters::Subscriber<metal_detector_msgs::Coil> md_sub_;
        tf::TransformListener tf_;
        tf::MessageFilter<metal_detector_msgs::Coil> * tf_filter_;
        ros::NodeHandle n_;
        std::string target_frame_;
        ros::Subscriber sub_move_handle;
        ros::Publisher minefound_pub;
        ros::Publisher pub_mine_pos;
      
        void move_handle();
        void msgCallback(const boost::shared_ptr<const metal_detector_msgs::Coil>& coil_ptr); 
        void move_handle_cmd_Callback(const std_msgs::Bool::ConstPtr& msg);
        void send_move_robot(double x, double y, double state); 
        void set_mine(double x, double y);  
};


void metal_detector::move_handle_cmd_Callback(const std_msgs::Bool::ConstPtr& msg)
{
     //cout<< "I heard: "<< msg->data << endl;
    if(msg->data)
    {
        can_move = true;
        min = minmin;
        max = min + del;
    }
}

void metal_detector::set_mine(double x, double y)
{
    ros::NodeHandle pn("~");
    std::string frame_id;
    pn.param<std::string>("frame_id", frame_id, "minefield");
    geometry_msgs::PoseStamped mine_pose;
    mine_pose.header.stamp = ros::Time::now();
    mine_pose.header.frame_id = frame_id;
    // We're actually only using a point x y...
    mine_pose.pose.position.x = x;
    mine_pose.pose.position.y = y;
    mine_pose.pose.position.z = 0.0;
    mine_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    ROS_INFO("Setting a mine on x:%lf y:%lf", x, y);

    pub_mine_pos.publish(mine_pose);
}
void metal_detector::send_move_robot(double x, double y, double state)
{
    std_msgs::Float64MultiArray mine_msg;
    mine_msg.data.resize(3);
    mine_msg.data[0] = x;
    mine_msg.data[1] = y;
    mine_msg.data[2] = state;
    minefound_pub.publish(mine_msg);
}


metal_detector::metal_detector(ros::NodeHandle *nh, actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *ac):nh_(nh),ac_(ac),tf_(),  target_frame_("minefield")
{
    can_move = false;
    coil_number = 0;
    minmin = -0.8;
    maxmax = 0.8;
    del = 0.2;
    min = -0.8;
    max = min + del;
    height = 0.00;
    speed = 0.1;
    acceleration = 0.1;
    md_sub_.subscribe(n_, "coils", 10);
    tf_filter_ = new tf::MessageFilter<metal_detector_msgs::Coil>(md_sub_, tf_, target_frame_, 10);
    tf_filter_->registerCallback( boost::bind(&metal_detector::msgCallback, this, _1) );
    sub_move_handle = n_.subscribe("/MOVEHANDLE_PUBLISHER", 1000, &metal_detector::move_handle_cmd_Callback,this);
    
    minefound_pub = nh_->advertise<std_msgs::Float64MultiArray>("Mine_position_publisher", 1000);
    pub_mine_pos = nh_->advertise<geometry_msgs::PoseStamped>("/HRATC_FW/set_mine", 1);
    move_handle();
} 

void metal_detector::msgCallback(const boost::shared_ptr<const metal_detector_msgs::Coil>& coil_ptr)
{
    if(can_move)
    {
    coil_number++;
    if( coil_number == 3)
    {
        min = min + del;
        max = max + del;
        move_handle();
        coil_number = 0;
    }


/*
    /////

//////*/
    geometry_msgs::PointStamped point_in;
    point_in.header.frame_id = coil_ptr->header.frame_id;
    point_in.header.stamp = coil_ptr->header.stamp;
    point_in.point.x = 0.0;
    point_in.point.y = 0.0;
    point_in.point.z = 0.0;
    
    geometry_msgs::PointStamped point_out;
    try
    {
        tf_.transformPoint(target_frame_, point_in, point_out);
        int ch1 = coil_ptr->channel[0];
        int ch2 = coil_ptr->channel[1];
        int ch1z = coil_ptr->zero[0];
        int ch2z = coil_ptr->zero[1];
        int num = ch2 - ch2z;
        int den = ch1 - ch1z;
        double beta = den != 0? double(num)/double(den) : 200.0;
        double signature = beta != 1? beta/(1-beta) : 100.0;        
        cout << coil_ptr->header.frame_id.c_str() << " BETA:" << beta << "  Signature:" << signature << endl;
        if(strcmp("middle_coil",coil_ptr->header.frame_id.c_str()) == 0)
        {
            cout << "MIDDLE COIL"<<endl;
            if(beta > 5.0)
            {
                cout << "MINE FOUND" << endl;
                set_mine(point_out.point.x,point_out.point.y);
             }
        }

        // Note that z is the position of the coil, not the position of the possible metal sample!
        /*ROS_INFO("Coil %s with data ch0 %d ch1 %d ch2 %d at x %f y %f z %f",
                coil_ptr->header.frame_id.c_str(),
                coil_ptr->channel[0],
                coil_ptr->channel[1],
                coil_ptr->channel[2],
                point_out.point.x,
                point_out.point.y,
                point_out.point.z);
*/
     }
     catch (tf::TransformException &ex)
     {
         ROS_WARN("Failure %s\n", ex.what());
     }
}    
}

void metal_detector::move_handle()
{
    if(max > maxmax)
    {
        min = 0;
        max = min + del;
        can_move = false;
        send_move_robot(0,0,0);
    }
    double position = min;
    int count = 0;
    while(ros::ok())// && count == 0)
    {
        count++;
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.header.stamp = ros::Time::now();
        goal.trajectory.joint_names.resize(2);
        goal.trajectory.points.resize(1);
        goal.trajectory.joint_names[0] = "upper_arm_joint";
        goal.trajectory.points[0].positions.push_back(height);
        goal.trajectory.points[0].velocities.push_back(speed);
        goal.trajectory.points[0].accelerations.push_back(acceleration);
        goal.trajectory.joint_names[1] = "arm_axel_joint";
        goal.trajectory.points[0].positions.push_back(position = position == minmin ? maxmax : minmin);
        goal.trajectory.points[0].velocities.push_back(speed);
        goal.trajectory.points[0].accelerations.push_back(acceleration);
        goal.trajectory.points[0].time_from_start = ros::Duration(2.7);
        goal.goal_tolerance.resize(4);
        goal.goal_tolerance[0].name = "upper_arm_joint";
        goal.goal_tolerance[0].position = 0.01;
        goal.goal_tolerance[1].name = "arm_axel_joint";
        goal.goal_tolerance[1].position = 0.01;
        goal.goal_time_tolerance = ros::Duration(0.5);

        ac_->sendGoal(goal);

        // Wait for the action to return
        bool finished_before_timeout = ac_->waitForResult(ros::Duration(30.0));

        if(!finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac_->getState();
            ROS_ERROR("Sweep -- %s!", state.toString().c_str());
        }
        cout << "MOVING" << endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sweep_node");
    ros::NodeHandle n;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/arm_controller/follow_joint_trajectory", true);
    ROS_INFO("Sweep -- Waiting for the action server to start...");
    ac.waitForServer();
    ROS_INFO("Sweep -- Got it!");
    metal_detector md(&n,&ac);

    ros::spin();
    return 0;
}
