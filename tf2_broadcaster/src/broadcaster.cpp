#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "broadcaster");

    static tf2_ros::TransformBroadcaster br;

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.child_frame_id = "world";
    transformStamped.header.frame_id = "base_link";

    double dist = 1.0;
    double rad = 1 * M_PI/180;
    

    ros::Rate publishing_rate(4);

    while(ros::ok())
    {
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.transform.translation.x = cos(rad) * 1;
        transformStamped.transform.translation.y = sin(rad) * 1;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);

        rad += 1 * M_PI/180;

        ros::spinOnce();
        publishing_rate.sleep();
    }
    
};