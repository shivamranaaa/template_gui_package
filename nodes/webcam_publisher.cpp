#include <iostream>
#include <cstdio>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

const std::string FRAME_SEPARATOR = "\xFF\xD8\xFF\xD9"; // Unique marker separator

// Helper function to search for the separator
size_t find_separator(const std::vector<char> &data, const std::vector<char> &separator) {
    auto it = std::search(data.begin(), data.end(), separator.begin(), separator.end());
    return (it != data.end()) ? std::distance(data.begin(), it) : std::string::npos;
}

	int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "frame_publisher");
    ros::NodeHandle nh;

    // Create a ROS publisher for the frame stream
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("webcam/image", 1);

    // Command to execute the frame producer
    const char *command = "/code_drive/src/kamikaze_main/pj_tflite_track_deepsort/build/main";

    // Open pipe to execute the command
    FILE *pipe = popen(command, "r");
    if (!pipe) {
        std::cerr << "Error: Failed to open pipe for command execution." << std::endl;
        return 1;
    }

    // Buffer for reading binary data from the pipe
    std::vector<char> buffer(8192);
    std::vector<char> frameData;
    std::vector<char> separatorChars(FRAME_SEPARATOR.begin(), FRAME_SEPARATOR.end());
    size_t separatorLength = separatorChars.size();

    // Main loop to read frames from the pipe and publish as ROS messages
    while (ros::ok()) {
        size_t bytesRead = fread(buffer.data(), 1, buffer.size(), pipe);
        if (bytesRead <= 0) break;

        // Append new data to frameData

        frameData.insert(frameData.end(), buffer.begin(), buffer.begin() + bytesRead);

        // Process frames within the accumulated data
        size_t separatorPos;
        while ((separatorPos = find_separator(frameData, separatorChars)) != std::string::npos) {
            std::vector<char> singleFrame(frameData.begin(), frameData.begin() + separatorPos);
            frameData.erase(frameData.begin(), frameData.begin() + separatorPos + separatorLength);

            try {
                // Decode the frame to an OpenCV Mat object
                cv::Mat image = cv::imdecode(cv::Mat(singleFrame), cv::IMREAD_COLOR);
                if (image.empty()) {
                    ROS_WARN("[frame_publisher]: Skipped - Empty image decoded.");
                    continue;
                }

                // Convert OpenCV image to ROS message
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

                // Publish the image message
                pub.publish(msg);
                ros::spinOnce();
            } catch (cv::Exception &e) {
                std::cerr << "cv::Exception: " << e.what() << std::endl;
                continue;
            }
        }
    }

    // Close the pipe
    int returnCode = pclose(pipe);
    if (returnCode != 0) {
        std::cerr << "Error: Failed to execute the command. Return code: " << returnCode << std::endl;
        return 1;
    }

    return 0;
}

