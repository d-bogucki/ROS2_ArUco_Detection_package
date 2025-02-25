//===========================================================================================================================================================================
// This ROS2 Node subscribes to the compressed image message published by the ESP32-CAM board and uses OpenCV to just show the image on the screen.
// Additionally the node logs the FPS and the image size
//===========================================================================================================================================================================

// Headers:
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp> 

using std::placeholders::_1;

class OpenCV_Subscriber : public rclcpp::Node
{
public:
  OpenCV_Subscriber() : Node("opencv_subscriber_node")
  {
    // BEST-EFFORT SUBSCRIPTION:
    subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", rclcpp::SensorDataQoS(), std::bind(&OpenCV_Subscriber::image_callback, this, _1)); 

    last_callback_time = (this)->now().nanoseconds()/1000000; 
  }

private:
  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received an image of size: %ld bytes", msg->data.size());
    try
    { 
      // Get the current time:
      rcl_time_point_value_t current_time = (this)->now().nanoseconds()/1000000; // [ms]
      
      // rcl_time_point_value_t is a typedef derived from an int64_t
      rcl_time_point_value_t elapsed_time = (current_time - last_callback_time); // [ms]

      last_callback_time = current_time;

      // Camera FPS:
      int8_t fps = (1.0/elapsed_time)*1000;

      // Log the elapsed time:
      RCLCPP_INFO(this->get_logger(), "Time since last callback: %ld [ms]", elapsed_time);
      RCLCPP_INFO(this->get_logger(), "FPS: %d", fps);      

      // Extracting a cv::Mat image object from the cv_bridge:
      cv::Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      //cv::Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;

      if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Decoded image is empty, likely due to packet loss.");
            return;
        }


      //cv::Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
      /*
      cv_bridge "toCvCopy" function description:
      Convert a sensor_msgs::msg::Image (or CompressedImage) to an OpenCV-compatible CvImage,
      copying the image data.

      Prototype:
      CvImagePtr toCvCopy(const sensor_msgs::msg::Image::ConstSharedPtr &source,
                          const std::string &encoding = std::string());
      Parametri:
      -) "source" --> A shared_ptr to a sensor_msgs::msg::Image message
      -) "encoding" --> The desired encoding of the image data, one of the following strings:
                        - "mono8"
                        - "bgr8"
                        - "rgb8"
                        - "rgba8"
                        - "mono16"                    
                        oss: if "encoding" is the empty string (default), the returned CvImage
                        has the same encoding as "source"
      

      // "CvIMage" class:
      Image message class that is interoperable with sensor_msgs/Image (or CompressedImage) but 
      uses a more convenient cv::Mat representation for the image data:

      class CvImage
      {
      public:
        std_msgs::msg::Header header; // ROS header
        std::string encoding;         // Image encoding ("mono8", "bgr8, etc.")
        cv::Mat image;                // Image data for use with OpenCV (ecco perche' quel "->" nel codice!)

        // list of constructors 
        ...
        // list of methods like: (to convert this message to a ROS sensor_msgs:msg::Image(or CompressedIMage)

        .toImageMsg()
        .toCompressedImageMsg()
        ...      
      };      
      */
      cv::imshow("Camera", frame);
      cv::waitKey(1);  // 1 ms wait 


    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;

  // Timestamp of the last callback
  rcl_time_point_value_t last_callback_time;
};


int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpenCV_Subscriber>());
  rclcpp::shutdown();
  return 0;
  
}
