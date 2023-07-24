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

This is for collecting data for hand-eye calibration.

- Automatically move robot and collect video data. (aruco marker should be visible at all time)

First run record_video_with_gripper_move.py in a terminal to start recording and then run joint.py in a seperate terminal to start to move the gripper by designed paths

```bash
python record_video_with_gripper_move.py
```

```bash
python joint.py
```

After collecting video data, run [convert_cameraPose2instant_data](nerf/instant-ngp/convert_cameraPose2instant_data) to convert video data to instant-ngp style input. The camera position is calculated through calibration.

```bash
python convert_cameraPose2instant_data.py --INPUT_DATA_DIR=$PROCESSED_DATA_PATH  --CALIBRATION_DATA_DIR=$CALIBRATION_DATA_DIR --DATA_TYPE='video'
```

or 

```bash
sh nerf/instant-ngp/convert_data.sh
```
This is for collecting data for 3D reconstruction of indoor scene based on instant-ngp.
