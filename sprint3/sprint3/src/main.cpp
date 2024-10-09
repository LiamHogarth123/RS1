#include "rclcpp/rclcpp.hpp"
#include "laserprocess.hpp"

int main(int argc, char**argv) {
    rclcpp::init(argc, argv);

    // create an instance of your node
    auto node = std::make_shared<rclcpp::Node>("laserProcessingNode");

    laserProcess processor(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
}