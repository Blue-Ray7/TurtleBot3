#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


void detect_blue_squares(cv_bridge::CvImagePtr) {

}


void detect_red_triangles(cv_bridge::CvImagePtr) {

}


void cameraCallback(const sensor_msgs::ImageConstPtr &cam_msg) {

    cv_bridge::CvImagePtr cv_img_ptr;

    try
    {   
        // Convert ROS image message to openCV image format
        cv_img_ptr = cv_bridge::toCvCopy(cam_msg, sensor_msgs::image_encodings::BGR8);

        detect_blue_squares(cv_img_ptr);
        detect_red_triangles(cv_img_ptr);

    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    
}


void laserCallback(const sensor_msgs::LaserScanConstPtr &laser_msg) {

    sensor_msgs::LaserScan laserScan = *laser_msg;

}


int main(int argc, char **argv){

    ros::init(argc, argv, "firefighter");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // Create tf2 buffer and listener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;
    tfListener = new tf2_ros::TransformListener(tfBuffer);


    // Create Subscribers
    image_transport::Subscriber imageSubscriber = it.subscribe("/camera/image", 1, cameraCallback);
    ros::Subscriber laserSubscriber = nh.subscribe("/scan", 1, laserCallback);

    // Create Publishers
    ros::Publisher blueSquarePublisher = nh.advertise<visualization_msgs::Marker>("/blue_square_pose", 1);
    ros::Publisher redTrianglePublisher = nh.advertise<visualization_msgs::Marker>("/red_triangle_pose", 1);

    ros::spin();

    // Free memory
    delete tfListener;
}