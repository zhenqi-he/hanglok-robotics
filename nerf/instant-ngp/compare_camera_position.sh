export COLMAP_JSON_DIR='/Users/zhenqihe/Desktop/横乐/hanglok-robotics/nerf/processed_07_06_18_40/transforms.json'
export INPUT_DATA_DIR='/Users/zhenqihe/Desktop/横乐/hanglok-robotics/nerf/processed_07_06_18_40'
export CALIBRATION_DATA_DIR='/Users/zhenqihe/Desktop/横乐/hanglok-robotics/calibration/saved_07_06_16_38'

python /Users/zhenqihe/Desktop/横乐/hanglok-robotics/nerf/instant-ngp/compare_estimated_cameraPosition.py --COLMAP_JSON_DIR=$COLMAP_JSON_DIR --INPUT_DATA_DIR=$INPUT_DATA_DIR --CALIBRATION_DATA_DIR=$CALIBRATION_DATA_DIR