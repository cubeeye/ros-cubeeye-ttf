# CUBE EYE camera ROS driver(TTF device)
These are packages for using TTF device with ROS
- #### MDC200DW

Please refer to our website for detailed product specifications. [http://www.cube-eye.co.kr](http://www.cube-eye.co.kr)



## Installation Instructions

The following instructions support ROS Kinetic, on Ubuntu 16.04.

### Step 1 : Install the ROS distribution
- #### Install ROS Kinetic, on Ubuntu 16.04

### Step 2 : Install driver
- #### Create a catkin workspace
```bash
$mkdir –p ~/catkin_ws/src
$cd ~/catkin_ws/src/
Copy the driver source to the path(catkin_ws/src)
```

- #### driver build
```bash
$catkin_init_workspace
$cd ..
$catkin_make clean
$catkin_make -DCMAKE_BUILD_TYPE=Release
```

- #### Setting environment
$source env_lib.sh
$sudo ttfinstall.sh

## Usage Instructions

Connect the camera power and execute the following command

```bash
$roslaunch cubeeye_ttf depth_camera.launch
```

#### Topics
- /cubeeye/ttf/amplitude_raw : IR Image
- /cubeeye/ttf/depth_raw : Depth Image
- /cubeeye/ttf/points : Point Cloud Image

#### Operating Test
```bash
$rqt
/cubeeye/ttf/amplitude_raw, /cubeeye/ttf/depth_raw
```
<p align="center"><img width=100% src="https://user-images.githubusercontent.com/18589471/67836365-c9ef1880-fac2-11e9-84c6-d5cbc6747d90.PNG"/></p>

```bash
$rosrun rviz rviz
Fixed Frame : pcl
PointCloud2 : /cubeeye/ttf/points
```
<p align="center"><img width=100% src="https://user-images.githubusercontent.com/18589471/67836438-028ef200-fac3-11e9-9ba4-84c60d19c32d.PNG"/></p>

#### Using Dynamic Reconfigure Params
```bash
$rosrun rqt_reconfigure rqt_reconfigure
```

<p align="center"><img width=100% src="https://user-images.githubusercontent.com/18589471/67836483-1a667600-fac3-11e9-805e-a9eeffc82c95.PNG"/></p>
