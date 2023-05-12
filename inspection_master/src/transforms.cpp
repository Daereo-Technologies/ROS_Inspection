#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

const float frequency = 100;
int main(int argc, char** argv){
    ros::init(argc, argv, "pointcloud_tf2_broadcaster");
    ros::NodeHandle nh;
    ros::Rate rate(frequency);

    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped trans;

    while(ros::ok()){
        trans.header.stamp = ros::Time::now();
        trans.header.frame_id = "map";
        trans.child_frame_id = "base_link";

        trans.transform.translation.x = 0;
        trans.transform.translation.y = 0;
        trans.transform.translation.z = 0;

        trans.transform.rotation.x = 0;
        trans.transform.rotation.y = 0;
        trans.transform.rotation.z = 0;
        trans.transform.rotation.w = 1;

        tf_broadcaster.sendTransform(trans);

        ros::spinOnce();
        rate.sleep();
    }

  return 0;
}


