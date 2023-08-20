#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv) {

    // Initialize the ROS node
    ros::init(argc, argv, "publisher");

    // Create Master node
    ros::NodeHandle nh;
    
    // Create a publisher object
    ros::Publisher pub = nh.advertise<std_msgs::String>("/hello_ros", 10);

    // Set the loop rate in Hz
    ros::Rate publishing_rate(10.0); // 10 Hz

    while (ros::ok())
    {
        // Create a message object and set message data
        std_msgs::String message;
        message.data = std::string("Hello ROS");

        ROS_INFO("[MESSAGE iSENT] %s", message.data.c_str());

        // Publish the message
        pub.publish(message);

        // Spin once to let the ROS node process the message callbacks
        ros::spinOnce();

        // Sleep for the remaining time to achieve the desired loop rate
        publishing_rate.sleep();
    }
    

    return 0;
}