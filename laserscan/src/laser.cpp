#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>


ros::Publisher pub;

void callback(sensor_msgs::LaserScan scan)
{
    geometry_msgs::PointStamped point_in_cartesian;
    point_in_cartesian.header = scan.header;
    point_in_cartesian.point.z = 0; // 2D Motion

    double min_dist = std::numeric_limits<double>::max(); // Largest possible value for type "double"

    for(int i = 0; i < scan.ranges.size(); ++i) 
    {
        // Find the nearest point to the laser scanner and convert it to the cartesian form
        if(scan.ranges.at(i) < min_dist && scan.ranges.at(i) >= scan.range_min && scan.ranges.at(i) <= scan.range_max)
        {
            min_dist = scan.ranges.at(i); // Compute distance of the laser beam with index "i"
            point_in_cartesian.point.x = min_dist * cos(i * scan.angle_increment + scan.angle_min);
            point_in_cartesian.point.y = min_dist * sin(i * scan.angle_increment + scan.angle_min);
        }
    }
    pub.publish(point_in_cartesian);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_reader");
    ros::NodeHandle nh;

    // Listen to laserscan data from topic "/scan"
    ros::Subscriber sub = nh.subscribe("/scan", 10, callback);

    // Publish point cartesian coordinates to topic "/nearest_point"
    pub = nh.advertise<geometry_msgs::PointStamped>("/nearest_point", 10);

    ros::spin();
}