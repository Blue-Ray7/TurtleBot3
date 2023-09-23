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

    // Add blur effect to image
    cv::Mat blurred_image;
    cv::GaussianBlur(cv_img_ptr->image, blurred_image, cv::Size(57,57), 0);

    // Convert back to ROS message which can be published
    imagePublisher.publish(cv_bridge::CvImage(cv_img_ptr->header, sensor_msgs::image_encodings::BGR8, blurred_image).toImageMsg());

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