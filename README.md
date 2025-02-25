# Description
This repository contains a package with 3 ROS2 nodes, all the nodes suscribe to a CompressedImage ROS message.
- The "opencv_subscriber" node simply displays the images on the screen and logs the FPS and image size
- The "esp32_cam_calibration" node performs an interactive camera calibration task given a 2D chessboard target
- The "aruco_detection_and_pose_estimation" node detects the ArUco markers in the frame and estimates the marker to camera pose. Then the node broadcasts the pose using the tf2 library.

More details soon.. DB
