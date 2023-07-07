# Hand-Eye Calibration for Intel D435i depth camera and Jaka

This is a python-version implement of hand-eye calibration (shown on the following figure) based on Jaka robot and Intel D435i depth camer

<p align="center">
  <img src="./example.png" />
</p>

## 1, Collect serval groups of gripper position and coresponding image
Run [record_robot_position.py]('https://github.com/zhenqi-he/hanglok-robotics/blob/master/record_robot_position.py') and press 1 to go on recording another image for another position.

Please move the gripper for each time. The system will not take the next image if the position of gripper does not change.

```bash
python record_robot_position.py
```
