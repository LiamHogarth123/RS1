#include <opencv2/opencv.hpp>
#include <cstdlib> // for getenv()

int main(int argc, char** argv) {
    // Define the path to the maps directory
    std::string home_path = std::getenv("HOME");  // Get the home directory
    std::string maps_path = home_path + "/maps/";  // Maps directory

    // Define the paths to the map.pgm and gazebo_sf_map.pgm files
    std::string map_file = maps_path + "sf_map.pgm";
    std::string gazebo_map_file = maps_path + "gazebo_sf_map.pgm";

    // Load the map.pgm file
    cv::Mat map_image = cv::imread(map_file, cv::IMREAD_GRAYSCALE);

    // Load the gazebo_map.pgm file
    cv::Mat gazebo_map_image = cv::imread(gazebo_map_file, cv::IMREAD_GRAYSCALE);

    // Scale down the sf_map.pgm image for display only
    cv::Mat scaled_map_image;
    double scale_factor_for_display = 0.5;  // Scale down to 50% of the original size
    cv::resize(map_image, scaled_map_image, cv::Size(), scale_factor_for_display, scale_factor_for_display);

    // Create windows to display the original images
    const std::string window_name_1 = "Map Image";
    const std::string window_name_2 = "Gazebo Map Image";
    const std::string window_name_3 = "Overlayed Image";
    cv::namedWindow(window_name_1, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(window_name_2, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(window_name_3, cv::WINDOW_AUTOSIZE);

    // Display the scaled map image and the original gazebo map image
    cv::imshow(window_name_1, scaled_map_image);
    cv::imshow(window_name_2, gazebo_map_image);

    // Apply transformations to the second image for overlay only
    cv::Mat transformed_gazebo_image = gazebo_map_image.clone();

    // Scale the cloned image
    double scale_factor = 1.6;
    cv::resize(transformed_gazebo_image, transformed_gazebo_image, cv::Size(), scale_factor, scale_factor);

    // Rotate the cloned image
    double angle = 92.5;
    cv::Point2f center(transformed_gazebo_image.cols / 2.0, transformed_gazebo_image.rows / 2.0);
    cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, angle, 1.0);
    cv::Rect bbox = cv::RotatedRect(center, transformed_gazebo_image.size(), angle).boundingRect();
    rotation_matrix.at<double>(0, 2) += bbox.width / 2.0 - center.x;
    rotation_matrix.at<double>(1, 2) += bbox.height / 2.0 - center.y;
    cv::warpAffine(transformed_gazebo_image, transformed_gazebo_image, rotation_matrix, bbox.size());

    // Expand the canvas to prevent cropping during translation
    int expand_x = 100;
    int expand_y = 50;
    cv::Mat expanded_image = cv::Mat::zeros(transformed_gazebo_image.rows + expand_y, transformed_gazebo_image.cols + expand_x, transformed_gazebo_image.type());
    transformed_gazebo_image.copyTo(expanded_image(cv::Rect(expand_x / 2, expand_y / 2, transformed_gazebo_image.cols, transformed_gazebo_image.rows)));

    // Translate the image
    cv::Mat translation_matrix = (cv::Mat_<double>(2, 3) << 1, 0, -70, 0, 1, -1);
    cv::warpAffine(expanded_image, expanded_image, translation_matrix, expanded_image.size());

    // Scale down the sf_map.pgm for overlaying
    cv::Mat scaled_map_image_for_overlay;
    cv::resize(map_image, scaled_map_image_for_overlay, cv::Size(), scale_factor_for_display, scale_factor_for_display);

    // Ensure proper scaling of the transformed image without distortion (preserving aspect ratio)
    int original_width = expanded_image.cols;
    int original_height = expanded_image.rows;
    int target_width = scaled_map_image_for_overlay.cols;
    int target_height = scaled_map_image_for_overlay.rows;
    double aspect_ratio_original = static_cast<double>(original_width) / original_height;
    double aspect_ratio_target = static_cast<double>(target_width) / target_height;
    int new_width, new_height;
    if (aspect_ratio_original > aspect_ratio_target) {
        new_width = target_width;
        new_height = static_cast<int>(target_width / aspect_ratio_original);
    } else {
        new_height = target_height;
        new_width = static_cast<int>(target_height * aspect_ratio_original);
    }
    cv::Mat properly_scaled_gazebo_image;
    cv::resize(expanded_image, properly_scaled_gazebo_image, cv::Size(new_width, new_height));

    // Center the image within the target overlay size
    cv::Mat centered_gazebo_image = cv::Mat::zeros(scaled_map_image_for_overlay.size(), properly_scaled_gazebo_image.type());
    int offset_x = (target_width - new_width) / 2;
    int offset_y = (target_height - new_height) / 2;
    properly_scaled_gazebo_image.copyTo(centered_gazebo_image(cv::Rect(offset_x, offset_y, new_width, new_height)));

    // Overlay the scaled sf_map.pgm and the properly scaled gazebo_sf_map.pgm with transparency
    cv::Mat overlay_image;
    double alpha = 0.5; // Set transparency level
    cv::addWeighted(scaled_map_image_for_overlay, alpha, centered_gazebo_image, 1 - alpha, 0, overlay_image);

    // Show the overlayed image in a new window
    cv::imshow(window_name_3, overlay_image);

    // Wait indefinitely until a key is pressed
    cv::waitKey(0);

    return 0;
}
