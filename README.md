# Pepperl+Fuchs SmartRunner Explorer Driver

This is the ROS2 driver for the Pepperl+Fuchs SmartRunner Explorer.
It offers ROS2 `Node` interfaces to the Robot Operating System (<http://www.ros.org>).

## Sensor information

The following Pepperl+Fuchs sensors are supported by this ROS driver.

### SmartRunner 3-D Explorer

The SmartRunner Explorer generates high-precision 3-D point cloud images in addition to 2-D images.
It is optionally available with stereo vision technology or time-of-flight (ToF) technology.
As a raw data sensor, the vision sensor is suitable for a wide variety of applications.

Official Website: https://www.pepperl-fuchs.com/global/en/smartrunner_3-d.htm

Datasheet (en): https://files.pepperl-fuchs.com/webcat/navi/productInfo/doct/tdoct7242a_eng.pdf

### SmartRunner Explorer

The SmartRunner Explorer is based on the innovative SmartRunner technology and outputs both height profiles and area
images. SmartRunner technology combines the light-sectioning method for acquiring height profiles with the acquisition
of area images via the integrated area illumination. In the light section method, a laser line is projected onto an
object. This is captured at a specific angle by a camera. A height profile is then created using the triangulation
principle. This laser technology enables reliable height profile recording on different surfaces.

Official Website: https://www.pepperl-fuchs.com/global/en/classid_9864.htm

Datasheet (en): https://files.pepperl-fuchs.com/webcat/navi/productInfo/pds/284586-100009_eng.pdf

## Usage with ROS

The ROS package `smartrunner_driver` consists of the driver library and three node executables, one for each supported SmartRunner sensor:
- `smartrunner_2d_node` for the SmartRunner Explorer
- `smartrunner_3d_stereo_node` for the SmartRunner 3-D Stereo Explorer
- `smartrunner_3d_tof_node` for the SmartRunner 3-D TOF Explorer

#### Supported platforms:

- Ubuntu 24.04 / ROS2 Jazzy

#### Published topics

The following standard ROS messages are supported. The messages contain the measured data.

- `scan` (`sensor_msgs/PointCloud2`)

#### Parameters

- `frame_id` - The frame-ID in the header of the published `sensor_msgs/PointCloud` messages
- `scanner_ip` - IP address or hostname of the sensor

#### Installation guide

1. Create a ROS workspace (https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) (e.g. in `<home>/ros2_ws`)
2. Clone the repository in the src folder of your ROS workspace.
```git clone https://github.com/PepperlFuchs/pf_smartrunner_ros2_driver.git```
3. Download the library from the Pepperl+Fuchs site.
   (https://www.pepperl-fuchs.com/global/en/classid_9866.htm?view=productdetails&prodid=117291#software)
4. Unzip the downloaded file and copy the folder "VsxSdk" in to the folder:
```<home>/ros2_ws/src/pf_smartrunner_ros_driver/smartrunner_driver/lib/```
4. Install .NET SDK.
   (https://learn.microsoft.com/en-us/dotnet/core/install/linux-ubuntu-install?tabs=dotnet9&pivots=os-linux-ubuntu-2404)
5. Change to the workspace directory.
```<home>/ros2_ws/```
6. Source the ROS2 setup script
```source /path/to/ros/jazzy/setup.bash```
7. Start the compilation.
```colcon build```

#### Launch the driver

1. Source the project-specific setup script
```source <home>/ros2_ws/install/setup.bash```
2. Set the IP-Address of the sensor and adjust all sensor parameters as needed in the launch file
   corresponding to the sensor you use, e.g. for the SmartRunner Explorer, edit
```<home>/ros2_ws/install/pepperl_fuchs_smartrunner/share/pepperl_fuchs_smartrunner/launch/smartrunner_2d.launch```
3. Run the launch file, by executing
```ros2 launch pepperl_fuchs_smartrunner smartrunner_2d.launch```
   Adjust the launch file name according to the sensor you use.
4. This starts `RViz2` (https://docs.ros.org/en/humble/p/rviz2/) and the driver.
5. The measure points coming from the sensor are shown in the `RViz2` window.

