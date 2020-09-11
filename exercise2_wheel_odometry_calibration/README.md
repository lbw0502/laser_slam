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

Suppose the robto does unifor motion in ![](https://latex.codecogs.com/gif.latex?%5CDelta%20t), 
