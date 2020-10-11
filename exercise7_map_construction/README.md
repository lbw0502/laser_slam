data miss

# Map Construction

## Occupancy Map

<dev align=center><img src=./doc/occ1.png></dev>


+ [occ2] is the inverse measurement model of lidar.

+ [occ3] represents the value of grid i at time t-1.

+ [occ4] represents the prior value of the grid (same for all grids)

<dev align=center><img src=./doc/occ5.png></dev>


**Inverse Measurement Model of Lidar**

<dev align=center><img src=./doc/occ6.png></dev>


+ the status of hitted grid is **Occupied**

+ the status of free grid is **Free**

+ the status of other grid is **Unknown**

### Code
open `roscore` and `rviz`

`rosrun occupany_mapping occupany_mapping`

<dev align=center><img src=./doc/result1.png></dev>

<dev align=center><img src=./doc/result1_detail.png></dev>


## Count Model Map

+ for grid i, two variables are considered: misses(i), hits(i)

> + misses(i) means the number of times that a beam goes through grid i

> + hits(i) means the number of times that a beam hits grid i

+ when hits(i)/(misses(i)+hits(i)) is larger than a threshold, the grid i can be seen as **occupied**

+ when hits(i)/(misses(i)+hits(i)) is smaller than a threshold, the grid i can be seen as **free**

+ hits(i)/(misses(i)+hits(i)) is the maximum likelihood of the occupied probability of grid i

### Code

`rosrun count_model_mapping count_model_mapping`

<dev align=center><img src=./doc/result2.png></dev>

<dev align=center><img src=./doc/result2_detail.png></dev>


## TSDF(Trunecated Signed Distance Function) Map

<dev align=center><img src=./doc/tsdf1.png></dev>

+ defination of sdf:

<dev align=center><img src=./doc/tsdf2.png></dev>

+ defination of tsdf:

> [sdf1] represents the distance measured by lidar

> [sdf2] represents the ditance between the hitted grid and robot

<dev align=center><img src=./doc/tsdf3.png></dev>

> t is the truncation distance

+ Fusion of multiple observation (weighted average):

<dev align=center><img src=./doc/tsdf4.png></dev>

the surface of object can be obtained from TSDF field:

<dev align=center><img src=./doc/tsdf5.png></dev>

### Code

`rosrun tsdf_mapping tsdf_mapping`

<dev align=center><img src=./doc/result3.png></dev>

<dev align=center><img src=./doc/result3_detail.png></dev>
