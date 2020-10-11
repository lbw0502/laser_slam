# Laser Regiseration using Gauss-Newton Method

## Generate a likelihood field of laser data

<dev align=center><img src=./doc/likelihood.png></dev>

## Registeration based on optimization method

target function:

<dev align=center><img src=./doc/gn1.png></dev>


where

+ ![](./doc/gn2.png) represents pose of robot

+ ![](./doc/gn3.png) represents coordinate of i-th laser point

+ ![](./doc/gn4.png) represents the transformed coordinate of i-th laser point

<dev align=center><img src=./doc/gn5.png></dev>

+ ![](./doc/gn6.png) represents the value at the likelihood field

**H and b of GN:**

<dev align=center><img src=./doc/hb.png></dev>

<dev align=center><img src=./doc/si.png></dev>

![](./doc/si2.png) represents the derivative of likelihood field value with respect to position. Since the likelihood field is discrete, we have to use interpolation to solve it.

**Bilinear interpolation**

assume that there are 4 points on a plane

<dev align=center><img src=./doc/inter1.png></dev>

let ![](./doc/inter2.png) and ![](./doc/inter3.png)

the coresponding coordinates become:

<dev align=center><img src=./doc/inter4.png></dev>

base function of Lagrangian Interpolation:

<dev align=center><img src=./doc/inter5.png></dev>

so the interpolation function is:

<dev align=center><img src=./doc/inter6.png></dev>

plug x and y back to the function:

<dev align=center><img src=./doc/inter7.png></dev>

so we can obtain the derivative of x and y, respectively:

<dev align=center><img src=./doc/inter8.png></dev>

<dev align=center><img src=./doc/inter9.png></dev>


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

<dev align=center><img src=./doc/result.png></dev>

green path is the ground truth of robot trajectory

red path is the GN optimized trajectory.
