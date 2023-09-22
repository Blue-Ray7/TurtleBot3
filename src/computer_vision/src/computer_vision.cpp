#include <ros/ros.h>
#include <sensor_msgs/Image.h> 
#include <cv_bridge/cv_bridge.h>


ros::Publisher imagePublisher;


void CVTest(sensor_msgs::Image msg)
{
    cv_bridge::CvImagePtr cv_img_ptr;
    
    try
    {   
        // Convert ROS image message to openCV image format
        cv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    // Convert back to ROS message which can be published
    imagePublisher.publish(cv_img_ptr->toImageMsg());   
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "computer_vision");
    ros::NodeHandle nh;

    // Listen to camera input 
    ros::Subscriber imageSubscriber = nh.subscribe("/camera/image", 2, &CVTest);

    // Publish modified images to new topic
    imagePublisher = nh.advertise<sensor_msgs::Image>("/modified_image", 2);

    ros::spin();
}