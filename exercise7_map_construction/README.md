data miss

# Map Construction

## Occupancy Map

[occ1]

+ [occ2] is the inverse measurement model of lidar.

+ [occ3] represents the value of grid i at time t-1.

+ [occ4] represents the prior value of the grid (same for all grids)

[occ5]

**Inverse Measurement Model of Lidar**

[occ6]

+ the status of hitted grid is **Occupied**

+ the status of free grid is **Free**

+ the status of other grid is **Unknown**

### Code
open `roscore` and `rviz`

`rosrun occupany_mapping occupany_mapping`

[result1]

[result1_detail]



## Count Model Map

+ for grid i, two variables are considered: misses(i), hits(i)

> + misses(i) means the number of times that a beam goes through grid i

> + hits(i) means the number of times that a beam hits grid i

+ when hits(i)/(misses(i)+hits(i)) is larger than a threshold, the grid i can be seen as **occupied**

+ when hits(i)/(misses(i)+hits(i)) is smaller than a threshold, the grid i can be seen as **free**

+ hits(i)/(misses(i)+hits(i)) is the maximum likelihood of the occupied probability of grid i

### Code

`rosrun count_model_mapping count_model_mapping`

[result2]

[result2_detail]


## TSDF(Trunecated Signed Distance Function) Map

[tsdf1]

+ defination of sdf:

[tsdf2]

+ defination of tsdf:

< [sdf1] represents the distance measured by lidar

< [sdf2] represents the ditance between the hitted grid and robot

[tsdf3]

< t is the truncation distance

+ Fusion of multiple observation (weighted average):

[tsdf4]

the surface of object can be obtained from TSDF field:

[tsdf5]

### Code

`rosrun tsdf_mapping tsdf_mapping`

[result3]

[result3_detail]


