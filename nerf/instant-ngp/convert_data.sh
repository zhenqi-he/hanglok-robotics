export ORIGINAL_DATA_PATH='/Users/zhenqihe/Desktop/横乐/hanglok-robotics/nerf/saved_07_06_18_40'
export PROCESSED_DATA_PATH='/Users/zhenqihe/Desktop/横乐/hanglok-robotics/nerf/processed_07_06_18_40'
export CALIBRATION_DATA_DIR='/Users/zhenqihe/Desktop/横乐/hanglok-robotics/calibration/saved_07_06_16_38'

python data_preprocess.py --DATA_PATH=$ORIGINAL_DATA_PATH --TAR_PATH=$PROCESSED_DATA_PATH

# python convert_cameraPose2instant_data.py --INPUT_DATA_DIR=$PROCESSED_DATA_PATH   --CALIBRATION_DATA_DIR=$CALIBRATION_DATA_DIR