# ArUco Localization for ROS Noetic

This package detects and localizes **ArUco tags** in images from a specified ROS topic.  
Using provided **camera intrinsics**, it estimates the relative pose of the tag with respect to the camera.  

Built for **ROS Noetic**.

---

## Installation

1. Create and initialize a catkin workspace:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   ```

2. Clone this repository into the `src` folder:
   ```bash
   git clone https://github.com/your-username/aruco_localization.git
   ```

3. Build the workspace:
   ```bash
   cd ~/catkin_ws
   catkin build
   ```

4. Source the setup file:
   ```bash
   source devel/setup.bash
   ```

---

## Usage

### Launch with ROS topic input
If you want to localize tags from a topic stream:
```bash
roslaunch aruco_localization aruco_localization.launch
```

### Quick test with built-in camera
For testing directly on your computer using your built-in camera:
```bash
roslaunch aruco_localization homemade.launch
```
This will try to use your webcam and detect tags of type **DICT_4X4_50**.

---

## Notes
- Requires **ROS Noetic**.
- Make sure your camera intrinsics are set correctly in the launch file for accurate pose estimation.
