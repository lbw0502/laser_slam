rosbag miss

# Pose Graph Optimization using Gauss-Newton

## Pose Graph

<dev align=center><img src=./doc/pose_graph.png></dev>

+ green line can be viewed as prediction, which is obtained from odometry information (laser odometry, imu, wheel odometry...)

<dev align=center><img src=./doc/prediction.png></dev>

<dev align=center><img src=./doc/form1.png></dev>

<dev align=center><img src=./doc/form2.png></dev>

+ red line can be viewed as observation, whihc is obtained by loop detection.

<dev align=center><img src=./doc/observation.png></dev>

so the error function is defined as:

<dev align=center><img src=./doc/error.png></dev>


### Error function in optimization

<dev align=center><img src=./doc/opt1.png></dev>


### Jacobian Matrix in optimization

<dev align=center><img src=./doc/opt2.png></dev>


## Code

open `roscore` and `rviz`

```
cd exercise6_poseGraph_optimization
catkin_make
source devel/setup.bash
rosrun ls_slam ls_slam
```

optimization result is shown in rviz

<dev align=center><img src=./doc/result.png></dev>


before optimization: blue

after optimization: purple
