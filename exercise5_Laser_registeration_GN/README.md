# Laser Regiseration using Gauss-Newton Method

## Generate a likelihood field of laser data
[img likelihood field]

## Registeration based on optimization method

target function:

[img gn1]

where

+ [gn1] represents pose of robot

+ [gn2] represents coordinate of i-th laser point

+ [gn3] represents the transformed coordinate of i-th laser point

[gn4]

+ [gn5] represents the value at the likelihood field

**H and b of GN:**

[hb]

[si]

[si2] represents the derivative of likelihood field value with respect to position. Since the likelihood field is discrete, we have to use interpolation to solve it.

**Bilinear interpolation**

assume that there are 4 points on a plane

[inter1]

let [inter2] and [inter3]

the coresponding coordinates become:

[inter4]

base function of Lagrangian Interpolation:

[inter5]

so the interpolation function is:

[inter6]

plug x and y back to the function:

[inter7]

so we can obtain the derivative of x and y, respectively:

[inter8]
[inter9]


## Code
start `roscore` and `rviz`

play bag file

`rosbag play --clock odom.bag`

```
cd exercise5_Laser_registeration_GN
catkin_make
source devel/setup.bash
rosrun gaussian_newton_scanmatcher gaussian_newton_node
```

the result is shown in rviz:

[result]

green path is the ground truth of robot trajectory

red path is the GN optimized trajectory.
