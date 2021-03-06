# Wheel odometry calibration
### Differential wheel motion model
<div align=center><img src = ./doc/motion_model1.png></div>

+ ![](https://latex.codecogs.com/gif.latex?%5Comega_R%2C%5Comega_L%3A) angular velocity of wheel  
+ ![](https://latex.codecogs.com/gif.latex?v_R%2Cv_L%3A) line velocity of wheel  
+ ![](https://latex.codecogs.com/gif.latex?b%3D2d): baseline of two wheels  
+ ![](https://latex.codecogs.com/gif.latex?%5Comega): angular velocity of base center
+ ![](https://latex.codecogs.com/gif.latex?v): line velocity of base center

<div align=center><img src = ./doc/motion_model3.png></div>
<div align=center><img src = ./doc/motion_model2.png></div>

so we have
<div align=center><img src = ./doc/calibr1.png></div>
<div align=center><img src = ./doc/calibr2.png></div>



### Wheel odometry calibration

Suppose the robto does unifor motion in ![](https://latex.codecogs.com/gif.latex?%5CDelta%20t), considering the angular integration of wheel odometry,
<div align=center><img src = ./doc/calibr3.png></div>

With n observations, we can construct a least square problem
<div align=center><img src = ./doc/calibr4.png></div>

where ![](https://latex.codecogs.com/gif.latex?S_%7B%5Ctheta%7D) is the angular increment measured by laser scanner.

Once ![](https://latex.codecogs.com/gif.latex?J_%7B21%7D) and ![](https://latex.codecogs.com/gif.latex?J_%7B22%7D) is known, considering the position integration of wheel odometry,
<div align=center><img src = ./doc/calibr5.png></div>

With n observations, we can construct another least square problem
<div align=center><img src = ./doc/calibr6.png></div>

where ![](https://latex.codecogs.com/gif.latex?s_%7Bx%7D) and ![](https://latex.codecogs.com/gif.latex?s_%7By%7D) are position increment measured by laser scanner.

Once ![](https://latex.codecogs.com/gif.latex?J_%7B21%7D) and ![](https://latex.codecogs.com/gif.latex?J_%7B22%7D) and b is known,
<div align=center><img src = ./doc/calibr7.png></div>

### Code
**odom.txt** contains wheel odometry data  
data format: time stamp, left wheel angualr velocity, right wheel angular velocity  
**scan_match** contains laser scanner data  
data format: time stamp, position increment(x), position increment(y), angular incremnet  

**compile the code**  
**run**  
`./odom_calib`  
**result**
<div align=center><img src = ./doc/result.png></div>
