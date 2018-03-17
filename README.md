# car107
基于视觉和激光的移动机器人的室内定位导航，对地板砖的一个检测，结合imu和里程计进行定位。采用到类似扫地机器人的全覆盖算导航规划法。
# street-sweeper

## 路面地板缝隙精确定位

使用lsd+Hough+ransac检测

使用颜色+LBP进行错误剔除


![avatar](lsd1.png)

![avatar](lsd_hough.png)

![avatar](ok.png)

![avatar](5.png)


## 里程估计

轮式里程计+IMU+线的角度定位+地板砖定位

定位误差在3mm左右

![avatar](speed.png)

## 牛耕式遍历法的导航规划

![avatar](2.png)

![avatar](3.png)


