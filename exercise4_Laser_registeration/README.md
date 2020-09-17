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



