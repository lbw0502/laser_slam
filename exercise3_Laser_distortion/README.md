# Laser data Calibration

### Reason of Lidar distortion
When the lidar scan rate is low compared to its extrinsic motion, the motion distortion within the scans can not be neglected. In this case, standard ICP methods can't be used.

<dev align=center><img src=./doc/distortion1.png></dev>

### Use Odometry information to calibrate Laser data

+ time stamp of first laser beam and last laser beam are ![](https://latex.codecogs.com/gif.latex?t_s), ![](https://latex.codecogs.com/gif.latex?t_e)  

+ the time interval of two laser beam is ![](https://latex.codecogs.com/gif.latex?%5CDelta%20t)  

+ odometry data is stored in a vector and fully covers laser data  

+ for each laser beam, compute the current rotation and translation using interpolation of odometry data  

+ transform all laser point to a reference frame (usually the frame for first laser beam)

+ encapsule the whole laser data and publish


### Code

1. install `champion_nav_msgs` to system  
```
cd exercise3_Laser_distortion/src/champion_nav_msgs  
sudo su
source /opt/ros/kinetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic install
```

2. build and run  
```
cd exercise3_Laser_distortion
catkin_make
source devel/setup.bash
roslaunch LaserUndistortion LaserUndistortion.launch
cd exercise3_Laser_distortion/bag
rosbag play --clock laser.bag
```

3. result  

<dev align=center><img src=./doc/result1.png></dev>

<dev align=center><img src=./doc/result2.png></dev>

+ the raw laser data is shown with red  
+ the calibrated laser data is shown with green
