#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h> 
#include <cv_bridge/cv_bridge.h>

/** 
 *  Image_transport package provides specialized publishers and subscribers for images
    which offer several advantages over the normal ROS publisher.

 *  It allows images to be compressed/decompressed before transmission reducing the 
    computational load on the system. This can be especially important when working with 
    high-resolution images or when network bandwidth is limited.
    
 *  It also provides a way to easily switch between different image transport mechanisms, 
    such as raw, JPEG, or PNG. This can be useful when working with different types of
    cameras or when trying to optimize performance for a particular use case.

 **/

image_transport::Publisher imagePublisher;

void imageCallback(const sensor_msgs::ImageConstPtr & msg)
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
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber imageSubscriber = it.subscribe("camera/image", 2, imageCallback);

    imagePublisher = it.advertise("mod_image_transport", 2);

    ros::spin();
}