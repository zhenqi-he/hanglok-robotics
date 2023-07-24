# hanglok-robotics
## Quick Start

- [Hany-eye Calibration](./calibration/README.md)
- [Data](#data)


## Data
This section includes codes to manipulate Jaka Zu robot to collect image data.

- Collect image data and corresponding TCP position (After taking one image at certain position, manually remove the TCP to next designed position is required). During Collection, Press 0 to stop collection

  SAVE_DIR in line 22 refers to the path to store collected image data
```bash
python record_robot_position.py
```

- Automatically move robot and collect video data.

First run record_video_with_gripper_move.py in a terminal to start recording and then run joint.py in a seperate terminal to start to move the gripper by designed paths

```bash
python record_video_with_gripper_move.py
```

```bash
python joint.py
```
  


