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
