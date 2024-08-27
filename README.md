# Automated-Guided-Vehicle
This repository contains the submission for ENPM673 Project 4 -  Implementation of Lane Following, Stop Sign detection and Dynamic obstacle detection/avoidance on a mobile robot (Turtlebot3 Waffle) using ROS2 and OpenCV (Hardware + Gazebo)

![Video GIF](https://github.com/suhasnagaraj99/Automated-Guided-Vehicle/blob/main/demo.gif)

## Repository Structure
- **enpm673_final_proj**: Package for launching the Turtlebot in Gazebo.
- **group8**: Package containing required scripts for completing the objectives in Gazebo.
- **group8_real**: Package containing required scripts for completing the objectives using a real robot.

## Prerequisites
- Before running the code, ensure that you have the following installed:
  - ROS2 (recommended version: Humble)
  - Gazebo
  - OpenCV

## Setup Instructions

1. **Create a Workspace**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```
2. **Copy Packages**:
   - Paste the `enpm673_final_proj`, `group8`and `group8_real` packages into the src folder of your workspace.

3. **Update Model Path**:
   - In `yolo.py` and `haar_cascade.py` scripts of packages `group8`and `group8_real` update the correct model path.    
     
4. **Build and Source the Packages**:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```
   
## Running the Simulation

1. **Launch Gazebo with Custom World**:
   ```bash
   ros2 launch enpm673_final_proj enpm673_world.launch.py
   ```
   
2. **Run the horizon detection script**:
   - Run the script to detect the horizon level of the camera:
   ```bash
   ros2 run group8 detect_horizon
   ```
   
3. **Run the stop-sign detection script**:
   - Run the below line to use yolo model to detect stop sign:
   ```bash
   ros2 run group8 yolo
   ```
   OR
   - Run the below line to use haar cascade model to detect stop sign:
   ```bash
   ros2 run group8 haar
   ```
   
4. **Run the paper centroid detection script**:
   - Run the below line to detect the paper centroids (waypoints to follow):
   ```bash
   ros2 run group8 paper_centroid
   ```
   
5. **Run the optical flow script**:
   - Run the optical flow script for dynamic obstacle detection/avoidance:
   ```bash
   ros2 run group8 optical
   ```
   
6. **Run the main script**:
   - Run the master node:
   ```bash
   ros2 run group8 turtlebot_controller
   ```
   - Ensure all other nodes are running before running this node.
     
7. **Visualize**:
   - To visualuize the output from the nodes, run the following command and select the topic as `camera_feed`
   ```bash
   ros2 run rqt_image_view rqt_image_view
   ```

## Running on Real Robot

1. **Bring up the robot and ensure data is being publsihed to camera topic and OpenCR is correctly integrated**:
   - SSH into the turtlebot3 and run:
   ```bash
   ros2 launch turtlebot3_bringup robot.launch.py
   ```
2. **Set the ROS Domain ID**
   - Ensure both the robot and your system os is on same network and have same `ROS_DOMAIN_ID`. Set the `ROS_DOMAIN_ID` using the line:
   ```bash
   export ROS_DOMAIN_ID=$domain_id$
   ```

3. **Run the horizon detection script**:
   - Run the script to detect the horizon level of the camera:
   ```bash
   ros2 run group8_real detect_horizon
   ```
   
4. **Run the stop-sign detection script**:
   - Run the below line to use yolo model to detect stop sign:
   ```bash
   ros2 run group8_real yolo
   ```
   OR
   - Run the below line to use haar cascade model to detect stop sign:
   ```bash
   ros2 run group8_real haar
   ```
   
5. **Run the paper centroid detection script**:
   - Run the below line to detect the paper centroids (waypoints to follow):
   ```bash
   ros2 run group8_real paper_centroid
   ```
   
6. **Run the optical flow script**:
   - Run the optical flow script for dynamic obstacle detection/avoidance:
   ```bash
   ros2 run group8_real optical
   ```
   
7. **Run the main script**:
   - Run the master node:
   ```bash
   ros2 run group8_real turtlebot_controller
   ```
   - Ensure all other nodes are running before running this node.
     
8. **Visualize**:
   - To visualuize the output from the nodes, run the following command and select the topic as `camera_feed`
   ```bash
   ros2 run rqt_image_view rqt_image_view
   ```

## Troubleshooting:
- **Path Issues**: Ensure that paths are correctly replaced in the scripts.
- **Dependencies**: Verify that all required libraries are installed.
- **Additional Information**: For more information refer to the project description: [PDF](https://github.com/suhasnagaraj99/Automated-Guided-Vehicle/blob/main/Turtlebot_Challenge_Final.pdf). The Homography requirement was later removed. 
