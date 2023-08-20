#include <ros/ros.h>
#include <std_msgs/String.h>

void callback(std_msgs::String msg) {
    ROS_INFO("Received Message %s", msg.data.c_str());
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/hello_ros", 10, callback);
    ros::spin();

    return 0;
}