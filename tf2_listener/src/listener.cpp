#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf(tfBuffer);    

    ros::Rate publishing_rate(1);
    
    while (ros::ok())
    {
        geometry_msgs::TransformStamped transformation;
        try
        {
        transformation = tfBuffer.lookupTransform("world", "base_link", ros::Time::now());
        }
        catch(tf2::TransformException &ex)
        {
            
            continue;
        }

        ros::spinOnce();
        publishing_rate.sleep();
    }
    
    
    

}