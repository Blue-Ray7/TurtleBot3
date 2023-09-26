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


// Set global variables
image_transport::Publisher imagePublisher;
ros::Publisher blueSquarePublisher;
ros::Publisher redTrianglePublisher;

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener *tfListener;

sensor_msgs::LaserScan laserScan;

void detect_blue_squares(cv::Mat hsv_frame) {

    /* Detect blue squares based on color and shape then find their poses (position + orientation) 
       from camera and laser scanner data.

       Methodology:  
       1) Define the lower and upper bounds for the blue color range using cv::Scalar
       2) Apply a color mask to the input image to extract blue regions using cv::inRange()
       3) Find contours of the blue regions using cv::findContours()
       4) Extract the position of the blue square markers by computing the center of the bounding 
          rectangle of each contour.

       5) Define an estimated camera angle, compute camera offset angle and the overall adjusted 
          camera angle
       6) Calculate laser ranges based on laser indices  
       
       7) Transform blue square coordinates from camera frame to map frame using tf2

    */ 

    // Create color mask
    cv::Mat blueMask;
    cv::Scalar blueLower(100, 50, 50); 
    cv::Scalar blueUpper(130, 255, 255);
    cv::inRange(hsv_frame, blueLower, blueUpper, blueMask);

    // Find contours of blue squares
    std::vector<std::vector<cv::Point>> blueContours;
    cv::findContours(blueMask, blueContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int count = 0;
    // Extract blue square positions
    for (const auto &contour : blueContours) {

        double area = cv::contourArea(contour);
        
        // Filter out noisy detections 
        if (area > 100) {

            count += 1;

            cv::Rect boundingRect = cv::boundingRect(contour);
            geometry_msgs::Point markerPoint;

            // Compute center of bounding rectangle
            markerPoint.x = boundingRect.x + boundingRect.width/2;
            markerPoint.y = boundingRect.y + boundingRect.height/2;;
            markerPoint.z = 0.0;

            ROS_INFO("markerPoint.x: %.2f", markerPoint.x);
            ROS_INFO("markerPoint.y: %.2f", markerPoint.y);

            // Define camera parameters
            double estimated_camera_fov = M_PI / 8;
            double angle_offset = ((markerPoint.x - 320) / 320);
            double adjusted_camera_fov = estimated_camera_fov * angle_offset;
            double final_camera_fov = fmod((2*M_PI + adjusted_camera_fov), (2 * M_PI));

            // Compute laser index
            int laserscan_index = static_cast<int> (final_camera_fov / laserScan.angle_increment);
            double laserscan_range = laserScan.ranges.at(laserscan_index);
            

            ROS_INFO("[BLUE] Final Angle: %.2f - Index: %i", final_camera_fov*180 / M_PI, laserscan_index);
        }

    }

}


void detect_red_triangles(cv::Mat hsv_frame) {

    /* See description above*/

}


void cameraCallback(const sensor_msgs::ImageConstPtr &cam_msg) {

    try
    {   
        /* HSV color space separates luma (image intensity) from chroma (color information)
           which makes it easier to detect objects based on their color. HSV is more illumination 
           invariant than RGB. This means that the color of an object in an image will remain the 
           same even if the lighting conditions change
        */
        cv::Mat frame = cv_bridge::toCvShare(cam_msg, "bgr8")->image;
        cv::Mat hsv_frame;
        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

        detect_blue_squares(hsv_frame);
        detect_red_triangles(hsv_frame);

    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

}


void laserScanCallback(const sensor_msgs::LaserScanConstPtr &laser_msg) {
    
    laserScan = *laser_msg;

}


int main(int argc, char **argv){

    ros::init(argc, argv, "firefighter");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // Create tf2 listener
    tfListener = new tf2_ros::TransformListener(tfBuffer);

    // Create Subscribers
    image_transport::Subscriber imageSubscriber = it.subscribe("camera/image", 1, cameraCallback);
   
    ros::Subscriber laserSubscriber = nh.subscribe("/scan", 1, laserScanCallback);

    // Create Publishers
    blueSquarePublisher = nh.advertise<visualization_msgs::Marker>("/blue_square_pose", 1);
    redTrianglePublisher = nh.advertise<visualization_msgs::Marker>("/red_triangle_pose", 1);

    ros::spin();

    delete tfListener;  // Free memory

    return 0;
}