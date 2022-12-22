#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char **argv){
   ros::init(argc, argv, "ref");

   ros::NodeHandle n;
   ros::Publisher ref_pub = n.advertise<geometry_msgs::PoseStamped>("uav/ref", 1000);
   ros::Rate loop_rate(100);

   float x = 0;
   float y = 0;
   float z = 2;

   float roll = 0;
   float pitch = 0;
   float yaw = 0;
   float yaw2 = 0;

   while (ros::ok()){

	geometry_msgs::PoseStamped ref_pose;

	ref_pose.pose.position.x = x; 
	ref_pose.pose.position.y = y; 
	ref_pose.pose.position.z = z; 

	ref_pose.pose.orientation.x = roll;
	ref_pose.pose.orientation.y = pitch;
	ref_pose.pose.orientation.z = yaw;
	ref_pose.pose.orientation.w = yaw2;

	ref_pose.header.stamp = ros::Time::now();
	ref_pose.header.frame_id = "uav_ref";


	ref_pub.publish(ref_pose);
	ros::spinOnce();
	loop_rate.sleep();
  }

  return 0;

}
