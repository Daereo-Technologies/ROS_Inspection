/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_ref;
void ref_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	current_ref.pose.position.x = msg -> pose.position.x;
	current_ref.pose.position.y = msg -> pose.position.y;
	current_ref.pose.position.z = msg -> pose.position.z;


	current_ref.pose.orientation.x = msg -> pose.orientation.x;
	current_ref.pose.orientation.y = msg -> pose.orientation.y;
	current_ref.pose.orientation.z = msg -> pose.orientation.z;
	current_ref.pose.orientation.w = msg -> pose.orientation.w;
	
	current_ref.header.frame_id = msg -> header.frame_id;
	//current_ref.header.stamp = msg -> header.stamp;
}

int main(int argc, char **argv)
{

    ROS_INFO("Control node online");
    ROS_INFO("Listening to waypoints...");
    ros::init(argc, argv, "ref_offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber ref_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav/ref", 100, ref_cb);
    //ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
    //        ("mavros/setpoint_position/local", 100);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 100);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
	} else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

	current_ref.header.stamp = ros::Time::now();
        local_pos_pub.publish(current_ref);
        //cmd_vel_pub.publish(current_ref);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


