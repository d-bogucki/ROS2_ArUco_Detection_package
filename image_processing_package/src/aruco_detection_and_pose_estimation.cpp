//===========================================================================================================================================================================
// This ROS2 Node subscribes to the compressed image message (published by the ESP32 micro-ROS node) and based on the camera calibration
// intrinsic parameters, saved in the .xml file, detects the ArUco Markers in the environment and broadcast the estimated pose using tf2 library 
// NOTE: The application has some absolute paths! change it accordingly to your file paths
//===========================================================================================================================================================================

// Headers:
#include <cstdio>
#include <functional>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp> 
#include <opencv2/aruco.hpp>
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;


class ArUco_detection_and_pose_estimation : public rclcpp::Node
{
public:
  ArUco_detection_and_pose_estimation() : Node("aruco_det_and_pose_est_node")
  {
    //======================================================================================
    // Parameters Initialization
    //======================================================================================
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50); 
    marker_length = 6.9/100; // [m]
    xml_path = "/home/dawidb/ros2_iron_LVM24_ws/src/image_processing_package/src/intrinsic_parameters.xml";

    // Reading Intrinsic camera parameters from .xml file:
    cv::FileStorage fs(xml_path, cv::FileStorage::READ);    
    fs["camera_matrix"] >> camera_Matrix;
    fs["distortion_coefficients"] >> dist_coeff;
    // Realeasing:
    fs.release();

    //======================================================================================
    // Subscription and Publishing Initialization
    //======================================================================================
    // Best-Effort Subscription:
    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "image/compressed", rclcpp::SensorDataQoS(), std::bind(&ArUco_detection_and_pose_estimation::image_callback, this, _1));
    
    // Broadcasting the marker pose:
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  }

private:
    //==========================================================================================
    // Configuration Parameters and Initialization:  
    //==========================================================================================
    // .xml file path:
    std::string xml_path;
    // Camera Matrix:
    cv::Mat camera_Matrix;
    // Distortion coefficients:
    cv::Mat dist_coeff;
    // Creating the Dictionary:
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    // Marker lenght:
    float marker_length;
    
    //==========================================================================================
    // Utility Functions:  
    //==========================================================================================
    geometry_msgs::msg::Quaternion opencv_rvec_to_ros_quaternion(cv::Vec3d &rvec){

        // Extracting omega (axis) and theta (angle) from rvec: (rvec = theta * omega)
        float theta = cv::norm(rvec);
        float omega_x = rvec[0]/theta;
        float omega_y = rvec[1]/theta;
        float omega_z = rvec[2]/theta;

        // Computing the quaternion components:
        float q_0 = std::cos(theta/2);
        float q_1 = std::sin(theta/2)*omega_x;
        float q_2 = std::sin(theta/2)*omega_y;
        float q_3 = std::sin(theta/2)*omega_z;

        geometry_msgs::msg::Quaternion quaternion;
        quaternion.x = q_1;
        quaternion.y = q_2;
        quaternion.z = q_3;
        quaternion.w = q_0;

        return quaternion;

    }

    //==========================================================================================
    // SUBSCRIPTION CALLBACK:  
    //==========================================================================================  
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        try
        {
        cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

        //cv::Mat img_copy;
        //img_copy = img.clone();

        // Vectors buffers:
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        // Marker detection:
        cv::aruco::detectMarkers(img, dictionary, corners, ids);

        // Text Parameters:
        // Font type:
        int font_type = 1; // FONT_HERSHEY_SIMPLEX
        // Font scale factor:
        double font_scale = 1;
        // Text Color:
        const cv::Scalar WHITE(255,255,255);
        const cv::Scalar BLACK(0,0,0);
        // Thickness of the lines of the text:
        int thickness = 1;          // default value
        // Line type:
        int line_type = cv::LINE_8; // default value
        // Text Size:
        int baseline = 0;
        cv::Size text_size = cv::getTextSize("text", font_type, font_scale, thickness, &baseline);

        // If at least one marker detected:
        if (ids.size() > 0) {

            // Marker drawing on the image:
            cv::aruco::drawDetectedMarkers(img, corners, ids);

            // Rotation and translation vectors buffers:
            std::vector<cv::Vec3d> rvecs, tvecs;

            // Pose estimating:
            cv::aruco::estimatePoseSingleMarkers(corners, marker_length, camera_Matrix, dist_coeff, rvecs, tvecs);

            int text_column = text_size.height;
            // Draw axis for each marker:
            for (int i=0; i<(int)ids.size(); i++) {

                // Axis Draw:
                cv::aruco::drawAxis(img, camera_Matrix, dist_coeff, rvecs[i], tvecs[i], 0.1);

                //==================================================================================
                // IMAGE TEXT:
                //==================================================================================

                // Quick algorithm for sorting the identified marker's ID in ascending order (to avoid text flickering)
                int min_index, id_buf;
                cv::Vec3d rvec_buf, tvec_buf;
                if (ids.size() > 1) {
                  for (int i=0; i < ((int)ids.size()-1); i++) {
                    min_index = i;

                    for (int j=(i+1); j < (int)ids.size(); j++) {
                      if (ids[j] < ids[min_index]) {
                        min_index = j;
                      }
                    }
                    // Change positions:
                    if (min_index != i) {
                      tvec_buf = tvecs[i];
                      rvec_buf = rvecs[i];
                      id_buf = ids[i];
                      tvecs[i] = tvecs[min_index];
                      rvecs[i] = rvecs[min_index];
                      ids[i] = ids[min_index];
                      tvecs[min_index] = tvec_buf;
                      rvecs[min_index] = rvec_buf;
                      ids[min_index] = id_buf;
                    }
                  }
                }
                
                std::ostringstream tvec_text, rvec_text, marker_id;
                tvec_text << "TVEC: " << tvecs[i] << " [m]";
                rvec_text << "RVEC: " << rvecs[i];
                marker_id << "ArUco Marker ID : " << ids[i]; 
               
                cv::putText(img, marker_id.str(), cv::Point(10,text_column), font_type, font_scale, BLACK, thickness, line_type); 
                cv::putText(img, tvec_text.str(), cv::Point(10,(text_column + text_size.height + 10)), font_type, font_scale, BLACK, thickness, line_type);  
                cv::putText(img, rvec_text.str(), cv::Point(10,(text_column + 2*text_size.height + 20)), font_type, font_scale, BLACK, thickness, line_type);  
                text_column += (3*(i+1)*(text_size.height) + 40);  

            }

            //==================================================================================
            // Marker Pose Broadcasting:
            //==================================================================================

            for (int i=0; i<(int)ids.size(); i++) {

              // From Rodrigues vector to ROS Quaternion:
              geometry_msgs::msg::Quaternion ros_quaternion = opencv_rvec_to_ros_quaternion(rvecs[i]);

              // Per adesso faccio una prova su un singolo marker:
              rclcpp::Time time = this->get_clock()->now();

              geometry_msgs::msg::TransformStamped marker_pose;
              marker_pose.header.stamp = time;
              marker_pose.header.frame_id = "camera";
              marker_pose.child_frame_id = "ArUco_Marker_" + std::to_string(ids[i]);
              marker_pose.transform.translation.x = tvecs[i][0];
              marker_pose.transform.translation.y = tvecs[i][1];
              marker_pose.transform.translation.z = tvecs[i][2];
              marker_pose.transform.rotation.x = ros_quaternion.x;
              marker_pose.transform.rotation.y = ros_quaternion.y;
              marker_pose.transform.rotation.z = ros_quaternion.z;
              marker_pose.transform.rotation.w = ros_quaternion.w;

              tf_broadcaster_->sendTransform(marker_pose);
            }
        }   

        cv::imshow("Marker Detection", img);
        char key = cv::waitKey(1);
        if (key == 27) {              // 27 is the ESC key
            RCLCPP_INFO(this->get_logger(), "Shutting down the node..");
            rclcpp::shutdown();
        }

        }
        catch(const std::exception& e)
        {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    
    }

    //==========================================================================================
    // Subscriber and Frame Broadcaster private members:  
    //==========================================================================================

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};


int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArUco_detection_and_pose_estimation>());
  rclcpp::shutdown();
  return 0;
}
