#include <sensor_msgs/msg/laser_scan.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory>
#include "nav_msgs/msg/path.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "opencv2/opencv.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


struct Point {
    double x, y;
};

struct Point_scan {
    double x, y, range, angle;
};



class laserProcess {
    public:

    laserProcess(std::shared_ptr<rclcpp::Node> node);

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr Msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);


    bool isCylinder(double arc_length, double angle_span);

    void draw_cylinder(double x, double y);
    

    double distance_between_points(Point_scan A, Point_scan B);

    void publishMarker(double x_local, double y_local);

    void publish_marker_array(std::vector<Point> points);
    void show_segments(std::vector<std::vector<Point_scan>> segments_);

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);


    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q);



    private:

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub2_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub2_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher;

    std::shared_ptr<rclcpp::Node> node_;  

    std::mutex lidar_locker;

    sensor_msgs::msg::LaserScan scan_data;
    nav_msgs::msg::Odometry odom;
    nav_msgs::msg::OccupancyGrid map;

    bool map_data_recieved;

    cv::Mat map_image;

    std::vector<std::vector<Point_scan>> segments;
    


};