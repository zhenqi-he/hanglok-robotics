export ORIGINAL_DATA_PATH='C:\Users\HP\Desktop\hzq\hanglok-robotics\calibration\saved_07_21_17_03'
export PROCESSED_DATA_PATH='C:\Users\HP\Desktop\hzq\hanglok-robotics\calibration\saved_07_21_17_58'
export CALIBRATION_DATA_DIR='C:\Users\HP\Desktop\hzq\hanglok-robotics\calibration\saved_07_21_17_58'

# python data_preprocess.py --DATA_PATH=$ORIGINAL_DATA_PATH --TAR_PATH=$PROCESSED_DATA_PATH

python convert_cameraPose2instant_data.py --INPUT_DATA_DIR=$PROCESSED_DATA_PATH   --CALIBRATION_DATA_DIR=$CALIBRATION_DATA_DIR --DATA_TYPE='video'