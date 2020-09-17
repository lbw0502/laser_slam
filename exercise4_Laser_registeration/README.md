# Laser Registeration algorithm

### ICP

<dev align=center><img src=./doc/icp1.png></dev>


Given two set of point cloud,

<dev align=center><img src=./doc/icp2.png></dev>

find rotation matrxi R and translation t to minimize

<dev align=center><img src=./doc/icp3.png></dev>


**when the corresponding point pairs are known**  

+ ![](./doc/icp4.png), ![](./doc/icp5.png), 

where ![](https://latex.codecogs.com/gif.latex?u_x) and ![](https://latex.codecogs.com/gif.latex?u_p) represent the center of point cloud X and point cloud P.

+ center the point cloud

<dev align=center><img src=./doc/icp6.png></dev>

<dev align=center><img src=./doc/icp7.png></dev>

+ the closed solution of ICP is 

<dev align=center><img src=./doc/icp8.png></dev>

**when the corresponding point pairs are unknown**  

+ find corresponding points(nearest points)

+ compute R and t according to matched points

+ apply R and t to source point cloud, compute the error

+ iterate until the error is smaller than a threshold


### IMLS(Implicit Moving Least Square) ICP

<dev align=center><img src=./doc/imls1.png></dev>

IMLS defines a function ![](https://latex.codecogs.com/gif.latex?I%5E%7BP_k%7D%28x%29) as an approximate distance of any point x in ![](https://latex.codecogs.com/gif.latex?%5Cmathbb%7BR%7D%5E3) to the implicit surface defined by the point cloud ![](https://latex.codecogs.com/gif.latex?P_k)

<dev align=center><img src=./doc/imls2.png></dev>

the weights ![](https://latex.codecogs.com/gif.latex?W_i%28x%29) are defined as:

<dev align=center><img src=./doc/imls3.png></dev>

and ![](https://latex.codecogs.com/gif.latex?n_i) is normal vector of point ![](https://latex.codecogs.com/gif.latex?p_i)

Now we want to localize the current scan ![](https://latex.codecogs.com/gif.latex?S_k) in previous scan ![](https://latex.codecogs.com/gif.latex?P_k). To do so, we want to minimize the sum of squared IMLS distances: ![](./doc/imls4.png). Due to exponential weights, we cannot approximate that nonlinear least-square optimization problem by a linear least-square one, as in ICP point to plane. Instead of minimizing that sum, we project every point ![](https://latex.codecogs.com/gif.latex?x_j) of ![](https://latex.codecogs.com/gif.latex?S_k) on the IMLS surface defined by ![](./doc/imls5.png) where ![](https://latex.codecogs.com/gif.latex?n_j) is the normal of the closest point to ![](https://latex.codecogs.com/gif.latex?x_j) and is a good approximation of the surface normal at projected point ![](https://latex.codecogs.com/gif.latex?y_i)

Now the new cost function is: 

<dev align=center><img src=./doc/imls6.png></dev>


### Code
`champion_nav_msgs` has to be installed
```
cd exercise4_Laser_registeration
catkin_make
source devel/setup.bash
```
**1. use IMLS to register the laser data**

`rosrun imlsMatcher imlsMatcher_node`

result is shown in rviz, 

red path: from odometry 

green path: from IMLS

<dev align=center><img src=./doc/result_imls.png></dev>

**2. use csm library(point-to-line) to register the laser data**

`rosrun csmMatcher csmMatcher_node`

result is shown in rviz,

red path: from odometry 

green path: from csm

<dev align=center><img src=./doc/result_csm.png></dev>

