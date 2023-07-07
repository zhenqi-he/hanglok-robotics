# Hand-Eye Calibration for Intel D435i depth camera and Jaka

This is a python-version implement of hand-eye calibration (shown on the following figure) based on Jaka robot and Intel D435i depth camer

<p align="center">
  <img src="./example.png" />
</p>

## 1, Collect serval groups of gripper position and coresponding image

Change the SAVA_DIR in line 20 of [record_robot_position.py]('https://github.com/zhenqi-he/hanglok-robotics/record_robot_position.py') and Run [record_robot_position.py]('https://github.com/zhenqi-he/hanglok-robotics/record_robot_position.py')  to start the collection process. After taking one image, the system will ask you whether to continue or not. Press 1 to go on recording another image for another position. Enter 0 if you want to stop the collection.

Please move the gripper for each time. The system will not take the next image if the position of gripper does not change.


```bash
python record_robot_position.py
```

## 2, Hand-Eye Calibration

In this system, we have n groups of gripper position information and corresponding image taken by the camera on the gripper at that position. As the position of marker (named Target in the figure) and the position of the robot base do not change so their corresponding transformation matrix also remain constant. We first calculate the matrix transform target coordinate to camera coodinate using detected corners of [aruco marker](aruco). And based on groups of datat to compute the matrix transforming camera coordinate to gripper coordinate hence get the matrix transforming camera coordinate to base coordinate.

```bash
sh calibrate.sh
```
