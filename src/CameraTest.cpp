#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <opencv2/opencv.hpp>

// Declare a subscriber for AprilTag detections
ros::Subscriber tag_sub;

// Callback function to handle detected tags
void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    if (msg->detections.empty()) {
        ROS_INFO("No AprilTags detected.");
        return;
    }

    for (const auto& detection : msg->detections) {
        int id = detection.id[0]; // AprilTag ID
        double x = detection.pose.pose.pose.position.x;
        double y = detection.pose.pose.pose.position.y;
        double z = detection.pose.pose.pose.position.z;

        ROS_INFO("Detected tag ID: %d, Position: [%.2f, %.2f, %.2f]", id, x, y, z);
    }
}

// Callback to process the RGB image
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv::Mat image = cv_bridge::toCvCopy(msg, "rgb8")->image;

        // Optional: Display the image (for debugging purposes)
        cv::imshow("RGB Image", image);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "CameraTesting");
    ros::NodeHandle nh;

    // Subscribe to the RGB image topic
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/xtion/rgb/image_color", 10, imageCallback);

    // Subscribe to the AprilTag detections
    tag_sub = nh.subscribe("/tag_detections", 10, tagCallback);

    ROS_INFO("CameraTest is running...");
    ros::spin();
    return 0;
}

