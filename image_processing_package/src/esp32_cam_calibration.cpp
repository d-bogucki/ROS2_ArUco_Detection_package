//===========================================================================================================================================================================
// This ROS2 node performs an interactive camera calibration task.
// In particular it subscribes to the compressed image topic (published by the ESP32-CAM micro-ROS Node), and captures a predefined number of images
// of a 2D Chessboard. Each capture is triggered after pressing the ENTER key. 
// Then the intrinsic parameters and distortion coefficient are saved in a .xml file
// NOTE: The application has some absolute paths! change it accordingly to your file paths
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
  OpenCV_Subscriber() : Node("esp32_cam_calibration")
  {
    //======================================================================================
    // Parameters Initialization
    //======================================================================================

    boardSize = cv::Size(9,6);
    squareSize = 25;
    grid_width = squareSize * (boardSize.width - 1);
    chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    nr_of_frames = 10;
    winSize = 11;
    aspect_ratio = 2;
    calibZeroTangentDist = false;
    calibFixPrincipalPoint = false;
    fixK1 = false;
    fixK2 = false;
    fixK3 = false;
    fixK4 = false;
    fixK5 = false;
    img_count = 0;

    flag = 0;   // flag parameter for the "cameraCalibration" function

    if(calibFixPrincipalPoint) flag |= cv::CALIB_FIX_PRINCIPAL_POINT; 
    if(calibZeroTangentDist)   flag |= cv::CALIB_ZERO_TANGENT_DIST;
    // if(aspect_ratio)            flag |= cv::CALIB_FIX_ASPECT_RATIO;
    if(fixK1)                  flag |= cv::CALIB_FIX_K1;
    if(fixK2)                  flag |= cv::CALIB_FIX_K2;
    if(fixK3)                  flag |= cv::CALIB_FIX_K3;
    if(fixK4)                  flag |= cv::CALIB_FIX_K4;
    if(fixK5)                  flag |= cv::CALIB_FIX_K5;

    // Best-Effort Subscription:
    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "image/compressed", rclcpp::SensorDataQoS(), std::bind(&OpenCV_Subscriber::image_callback, this, _1));  
  }

private:

//==========================================================================================
// Configuration Parameters and Initialization:  
//==========================================================================================
// CHESSBOARD INFORMATIONS:
cv::Size boardSize; // boardSize.width = 6, boardSize.height = 9
float squareSize;   // [mm], dimension of the edge of one square of the chessboard
float grid_width;
// flags for the "findChessboardCorners()" function:
int chessBoardFlags;

int nr_of_frames;   // The number of frames to use for the calibration
int winSize;        // Window size for the "cornerSubPix()" function 

// FLAGS FOR "cameraCalibration" function:
int flag;
float aspect_ratio; 
bool calibZeroTangentDist;   
bool calibFixPrincipalPoint; 
bool fixK1;                  
bool fixK2;                 
bool fixK3;                  
bool fixK4;                  
bool fixK5;    

// Variables for 3D/2D Points , Extrinsic and Intrinsic Parameters:
std::vector<std::vector<cv::Point2f>> imagePoints;
cv::Mat cameraMatrix, distCoeffs;
cv::Size imageSize;

int img_count; 

//==========================================================================================
// CALLBACK:  
//==========================================================================================

  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    try
    {
    // Getting the current frame from the topic:  
    cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      
    imageSize = img.size();

    //----------------------------------------------------------------------------------------------
    // RUN THE CALIBRATION IF ENOUGH IMAGES TAKEN:
    //----------------------------------------------------------------------------------------------

    if (img_count == nr_of_frames) {

        // Parameters buffers:
        std::vector<cv::Mat> rvecs, tvecs;  // rotation (Rodrigues) and translation vectors
        std::vector<float> reprojErrs;
        std::vector<std::vector<cv::Point3f> > objectPoints(1);
        double rms;                         // output of the "cameraCalibration()" function

        cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        //if( flag & cv::CALIB_FIX_ASPECT_RATIO )
        //cameraMatrix.at<double>(0,0) = aspect_ratio;

        // Calculate Board corner positions (objectPoints) in the calibration board coordinate space
        for (int i = 0; i < boardSize.height; ++i) {
            for (int j = 0; j < boardSize.width; ++j) {
                objectPoints[0].push_back(cv::Point3f(j*squareSize, i*squareSize, 0));
            }
        }
        // Resize of the Array of Arrays:
        objectPoints.resize(imagePoints.size(),objectPoints[0]); 

        // Calibration:
        rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, flag);

        bool ok = cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs);

        // Calibration results:
        if (ok) {
            RCLCPP_INFO(this->get_logger(), "Calibration succeeded! RMS = %f", rms);
        }
        else
            RCLCPP_INFO(this->get_logger(), "Calibration failed");
        //-----------------------------------------------------------------------------------------
        // SAVING THE INTRINSIC PARAMETERS:
        
        if (ok) {
            // .xml file path:
            std::string xml_file_path = "/home/dawidb/ros2_iron_LVM24_ws/src/image_processing_package/src/intrinsic_parameters.xml";

            cv::FileStorage fs(xml_file_path, cv::FileStorage::WRITE);

            fs << "camera_matrix" << cameraMatrix;
            fs << "distortion_coefficients" << distCoeffs;
            fs.release();
        }
        

        // Ho cambiato un attimo i testi sull'immagine, ancora non lo ho testato! perche' altrimenti
        // mi sovrascrive le immagini salvate e gli intrinsic/extrinsic parameters, dopo pero' riprova ecco
        std::string exiting_msg = "Saving the calibration parameters..";
        cv::Point text_origin_final(0, img.rows - 30);
        const cv::Scalar GREEN(0,255,0);
        cv::putText(img, exiting_msg, text_origin_final, 0, 1, GREEN, 1, cv::LINE_8);
        cv::imshow("Calibration", img);  
        cv::waitKey(5000);
        rclcpp::shutdown();
    }

    //----------------------------------------------------------------------------------------------
    // Finding the pattern in the current frame:
    //----------------------------------------------------------------------------------------------

    std::vector<cv::Point2f> inner_corners; // buffer for the found "inner corners" of the chessboard
    bool found;
    found = cv::findChessboardCorners(img, boardSize, inner_corners, chessBoardFlags);
    // found will be "true" if the pattern was found in the current frame

    // Improving the detected inner corners position:

    cv::Mat img_gray;   // buffer for gray image (in order to improve the algorithm)

    // criteria for termination of corner refinement:
    cv::TermCriteria criteria( cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.0001 );

    cv::Mat clean_img = img.clone();    // clean image for saving the calibration image

    // Changing the color to gray:
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

    if (found) {
        // Improved inner corners algorithm: 
        cv::cornerSubPix(img_gray, inner_corners, cv::Size(winSize,winSize), cv::Size(-1,-1), criteria);
        
        // Draw the corners:
        cv::drawChessboardCorners(img, boardSize, cv::Mat(inner_corners), found);
    }
    //----------------------------------------------------------------------------------------------
    // Image Description Text:
    //----------------------------------------------------------------------------------------------
    // Text to be drawn:
    std::string msg1 = "Press ENTER to capture the current frame";

    // Bottom-left corner of the text string in the image:
    cv::Point text_origin(0, img.rows - 30);  
    // Font type:
    int font_type = 0; // FONT_HERSHEY_SIMPLEX
    // Font scale factor:
    double font_scale = 1;
    // Thickness of the lines of the text:
    int thickness = 1;          // default value
    int baseline = 0;

    cv::Size textSize = cv::getTextSize(msg1, font_type, font_scale, thickness, &baseline);

    // Selecting the font scale based on the image frame size:
    if (img.cols >= textSize.width) font_scale = 1;
    else {
        font_scale = (1 - 1.2*(textSize.width - img.cols)/textSize.width);
        thickness = (1 - 1.2*(textSize.width - img.cols)/textSize.width);
    }    

    // Text Color:
    const cv::Scalar BLUE(255,0,0);
    const cv::Scalar YELLOW(0,255,255);
    
    // Line type:
    int line_type = cv::LINE_8; // default value
    
    if(img_count == 0) {
        cv::putText(img, msg1, text_origin, font_type, font_scale, BLUE, thickness, line_type);
    }
    if (img_count > 0 && img_count < nr_of_frames) {
        std::string msg2 = "Calibration Images Captured :" + std::to_string(img_count) + "/" + std::to_string(nr_of_frames);
        cv::putText(img, msg2, text_origin, font_type, font_scale, YELLOW, thickness, line_type);
    }

    //----------------------------------------------------------------------------------------------
    // CAPTURING AND SAVING THE IMAGE:
    //---------------------------------------------------------------------------------------------- 
    
    cv::imshow("Calibration", img);

    char key = cv::waitKey(1);

    // Saving the current frame:
    if (key == 13 && found) { // 13 is the key for "Enter"

        imagePoints.push_back(inner_corners);
        
        // File name:
        std::string fileName = "/home/dawidb/ros2_iron_LVM24_ws/src/image_processing_package/src/calibration_images/image_" + std::to_string(img_count + 1) + ".png"; // Save as .png

        // Image writing to file:
        bool success = cv::imwrite(fileName, clean_img); // Write the "clean_img" to a file

        // Checking for correct saving:
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Saved  %s", fileName.c_str());
            // If saved increase the image counter:
            img_count++;
        } else {
            RCLCPP_INFO(this->get_logger(), "Failer to save  %s", fileName.c_str());
        }
        
    
    } else if (key == 27) { // Press 'ESC' to quit
        RCLCPP_INFO(this->get_logger(), "Quitting from the application..");
        rclcpp::shutdown();
    }

    }

    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  
  }
 
  // Subscription private member:
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
};


int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpenCV_Subscriber>());
  rclcpp::shutdown();
  return 0;
  
}
