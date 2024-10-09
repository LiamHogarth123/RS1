#include "laserprocess.hpp"


laserProcess::laserProcess(std::shared_ptr<rclcpp::Node> node){
    sub1_ = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&laserProcess::lidarCallback, this, std::placeholders::_1));

    sub2_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 10, std::bind(&laserProcess::mapCallback, this, std::placeholders::_1));

    odom_subscriber_ = node->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&laserProcess::odom_callback, this, std::placeholders::_1));
    


    marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    marker_pub2_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

        

    map_data_recieved = false;
    node_ = node;
}    


//add odom call back


void laserProcess::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
 
    odom = *msg;
}



void laserProcess::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr Msg) {
    std::cout << "lidarCallback called" << std::endl;

    if (map_data_recieved) {
        std::cout << "Map data received, processing lidar data" << std::endl;
        std::lock_guard<std::mutex> lock(lidar_locker);
        const double min_angle = -1.5;  // -0.5 radians (~ -30 degrees)
        const double max_angle = 1.5;   //  0.5 radians (~ 30 degrees)
        double last_id = -1;
        std::vector<Point_scan> segment;
        Point_scan Next_point;
        Point_scan Current_Point;

        // Scan only within the specified angle range
        for (size_t i = 0; i < Msg->ranges.size() - 1; ++i) {  // Prevent out-of-bounds
            double angle = Msg->angle_min + i * Msg->angle_increment;
            double range = Msg->ranges[i];

            double angle2 = Msg->angle_min + (i+1) * Msg->angle_increment;
            double range2 = Msg->ranges[i+1];

            // Skip points outside the angle range
            if (angle < min_angle || angle > max_angle) continue;
            // Skip invalid ranges
            if (range < Msg->range_min || range > Msg->range_max || std::isnan(range)) continue;

            // Convert to Cartesian coordinates
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);

            double x2 = range2 * std::cos(angle2);
            double y2 = range2 * std::sin(angle2);

            Current_Point.x = x;
            Current_Point.y = y;

            Next_point.x = x2;
            Next_point.y = y2;

            // Check if points are close enough to be part of the same segment
            if (distance_between_points(Current_Point, Next_point) < 0.2) {
                segment.push_back(Current_Point);
                last_id = i + 1;
            } else {
                // If no adjacent point found, complete the segment
                if (!segment.empty()) {
                    segments.push_back(segment);
                    segment.clear();  // Start a new segment
                }
            }
        }

        if (!segment.empty()) {
            segments.push_back(segment);  // Push the final segment
        }

        show_segments(segments);

        bool cylinder_found = false;

        // The subtended angle (angle span) is the difference between the max and min angles
        for (int j = 0; j < segments.size(); j++) {
            double angle_span = segments.at(j).back().angle - segments.at(j).at(0).angle;
            double arc_length = 0;
            std::vector<cv::Point2f> cv_points;

            // for (int m = 0; m < segments.at(j).size() - 1; m++) {
            //     arc_length += std::hypot(segments.at(j).at(m+1).x - segments.at(j).at(m).x, 
            //                             segments.at(j).at(m+1).y - segments.at(j).at(m).y);
            // }

            for (const auto& point : segments.at(j)) {
                cv_points.push_back(cv::Point2f(point.x, point.y));
            }

            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(cv_points, center, radius);

            std::cout << "Radius: " << radius << std::endl;

            // Check if the calculated radius matches the expected cylinder radius
            if (0.125 < radius && radius < 0.175) {     
                std::cout << "Cylinder detected!" << std::endl;
                draw_cylinder(center.x, center.y);
                cylinder_found = true;
            } else {
                std::cout << "No cylinder detected." << std::endl;
            }
        }

if (!cylinder_found) {
    std::cout << "No cylinder found in any segment" << std::endl;
}    } else {
        std::cout << "Map data not received yet" << std::endl;
    }
    segments.clear();
}

bool laserProcess::isCylinder(double arc_length, double angle_span) {
    std::cout << "Checking if arc is a cylinder..." << std::endl;
    double radius = arc_length / angle_span;
    std::cout << "Estimated radius: " << radius << std::endl;

    const double expected_radius = 0.5;  // Expected radius for 1m diameter cylinder
    const double radius_tolerance = 0.05; // 10 cm tolerance

    if (std::abs(radius - expected_radius) < radius_tolerance) {
        std::cout << "Radius matches expected cylinder!" << std::endl;
        return true;
    }

    std::cout << "Radius does not match expected cylinder." << std::endl;
    return false;
}

void laserProcess::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    std::cout << "Map callback triggered" << std::endl;
    map = *msg;
    map_data_recieved = true;
    std::cout << "Map data stored" << std::endl;

    int width = msg->info.width;
    int height = msg->info.height;

    // Create an OpenCV image based on map dimensions
    cv::Mat map_image(height, width, CV_8UC1);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int index = x + y * width;
            int8_t occupancy_value = msg->data[index];
            
            if (occupancy_value == -1) {
                map_image.at<uchar>(height - y - 1, x) = 205; // Unknown
            } else if (occupancy_value == 0) {
                map_image.at<uchar>(height - y - 1, x) = 254; // Free space
            } else if (occupancy_value == 100) {
                map_image.at<uchar>(height - y - 1, x) = 0; // Occupied
            }
        }
    }
    std::cout << "Map image generated" << std::endl;
}

void laserProcess::draw_cylinder(double x, double y){
    std::cout << "Drawing cylinder at (" << x << ", " << y << ")" << std::endl;


    double robot_x = odom.pose.pose.position.x;
    double robot_y = odom.pose.pose.position.y;
    
    double robot_theta = get_yaw_from_quaternion(odom.pose.pose.orientation);

    double x_map = robot_x + (x * std::cos(robot_theta) - y * std::sin(robot_theta));
    double y_map = robot_y + (x * std::sin(robot_theta) + y * std::cos(robot_theta));


    publishMarker(x_map,y_map);


    int pixel_x = (x_map - map.info.origin.position.x) / map.info.resolution;
    int pixel_y = (y_map - map.info.origin.position.y) / map.info.resolution;


    // cv::Point center;
    // center.x = pixel_x;
    // center.y = pixel_y;

    cv::Point center;
    center.x = pixel_x;
    center.y = map_image.rows - pixel_y;

    double radius = 3;
    cv::Scalar cylinder_colour(0, 0, 255); // BGR value for red
    
    cv::Mat grayscaleMapImage = cv::imread("/home/liam/map.pgm", cv::IMREAD_GRAYSCALE);
    cv::cvtColor(grayscaleMapImage, map_image, cv::COLOR_GRAY2BGR);


    cv::circle(map_image, center, radius, cylinder_colour, -1); // Draw the node as a filled circle

    std::cout << "Cylinder drawn at pixel coordinates: (" << pixel_x << ", " << pixel_y << ")" << std::endl;

    cv::imwrite("/home/liam/Desktop/map_with_cylinder.png", map_image);
    std::cout << "Map image saved with cylinder" << std::endl;
}


 void laserProcess::publishMarker(double x_local, double y_local) {
        // Create a marker message
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";  // This should match the robot's frame
        marker.header.stamp = node_->now();

        marker.ns = "cylinder";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;  // Use SPHERE for the center point
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the position of the cylinder center in local (robot) coordinates
        marker.pose.position.x = x_local;
        marker.pose.position.y = y_local;
        marker.pose.position.z = 0.0;  // Assuming the cylinder is at ground level
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker
        marker.scale.x = 0.1;  // Diameter of the marker (adjust as necessary)
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        // Set the color (red in this case)
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;  // Alpha (transparency)

        // Publish the marker
        marker_pub_->publish(marker);
    }


    void laserProcess::show_segments(std::vector<std::vector<Point_scan>> segments_){
        std::vector<Point> segments_mid_points;

        std::cout << "printing number of segments ???????????????????" << std::endl;
        std::cout << segments_.size() << std::endl;

        for (size_t j = 0; j < segments_.size(); j++) {
            size_t mid_index = segments_.at(j).size() / 2;

            Point temp;
            temp.x = segments_.at(j).at(mid_index).x + -2;
            temp.y = segments_.at(j).at(mid_index).y + -0.5;            
            segments_mid_points.push_back(temp);
        }

        // publish_marker_array(segments_mid_points);
    }

    void laserProcess::publish_marker_array(std::vector<Point> points)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // Create 5 example markers (e.g., spheres)
        for (int i = 0; i < points.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";  // You can adjust this depending on your frame
            marker.header.stamp = node_->now();
            marker.ns = "my_namespace";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;  // Sphere marker
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Set the position of each sphere
            marker.pose.position.x = points.at(i).x;  // Example: spaced 1m apart
            marker.pose.position.y = points.at(i).y;
            marker.pose.position.z = 0.0;

            // Set orientation (no rotation)
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            // Set the scale of the marker
            marker.scale.x = 0.2;  // Radius of the sphere
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;

            // Set the color (RGB + alpha)
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;  // Green color
            marker.color.a = 1.0f;  // Fully opaque

            // Add marker to the array
            marker_array.markers.push_back(marker);
        }

        // Publish the MarkerArray
        marker_pub2_->publish(marker_array);
    }


double laserProcess::distance_between_points(Point_scan A, Point_scan B){
    std::cout << "Calculating distance between points" << std::endl;
    double dist = std::sqrt(std::pow(A.x - B.x, 2) + std::pow(A.y - B.y, 2));
    std::cout << "Distance: " << dist << std::endl;
    return dist;
}


    double laserProcess::get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
    {
        // Calculate yaw from quaternion
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }