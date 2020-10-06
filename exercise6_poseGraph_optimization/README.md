rosbag miss

# Pose Graph Optimization using Gauss-Newton

## Pose Graph

[pose_graph]

+ green line can be viewed as prediction, which is obtained from odometry information (laser odometry, imu, wheel odometry...)

[prediction]

[form1]

[form2]

+ red line can be viewed as observation, whihc is obtained by loop detection.

[observation]

so the error function is defined as:

[error]

### Error function in optimization

[opt1]

### Jacobian Matrix in optimization

[opt2]

## Code

open `roscore` and `rviz`

```
cd exercise6_poseGraph_optimization
catkin_make
source devel/setup.bash
rosrun ls_slam ls_slam
```

optimization result is shown in rviz

[result]

before optimization: blue

after optimization: purple
