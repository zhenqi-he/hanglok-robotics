import time
import pyrealsense2 as rs
import numpy as np
import cv2
import jkrc
from HanglokJKRC import HagJkrc
import logging
import pandas as pd
from camera import Camera
import os
from datetime import datetime
import cv2.aruco as aruco
from show_up_experiment.utils import *
import os
import math
import cv2.aruco as aruco
import json
import argparse
import glob
import random

def create_dir(dir):
    if not os.path.exists(dir):
        os.mkdir(dir)
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--calibration_image_dir', default=f'C://Users//HP//Desktop//hzq//hanglok-robotics//calibration//saved_07_06_16_38') 
    parser.add_argument('--calibration_json_dir', default=f'C://Users//HP//Desktop//hzq//hanglok-robotics//calibration//saved_07_06_16_38') 
    parser.add_argument('--save_dir', default=f'C://Users//HP//Desktop//hzq//hanglok-robotics//calibration//') 
    args = parser.parse_args()
 
    data_path = args.calibration_image_dir
    save_dir = args.save_dir
    data_json_path = args.calibration_json_dir
    position_txt_path = os.path.join(data_path,'position.txt')
    position_info = read_position(position_txt_path)
    # Camera Format, please refer to camera document
    fps, w, h = 30, 1280, 720

    # current time
    now = str(datetime.now())
    current_time = now[5:7]+'_'+now[8:10]+'_'+now[11:13]+'_'+now[14:16]
    # save path of images and position information
    create_dir(save_dir)
    SAVE_DIR = os.path.join(save_dir,current_time)
    create_dir(SAVE_DIR)
    SAVE_IMG_DIR = os.path.join(SAVE_DIR,'images')
    create_dir(SAVE_IMG_DIR)
    output_json_path = os.path.join(SAVE_DIR,'transforms.json')

    # Read Previous Camera Data
    transforms_json = json.load(open(data_json_path))
    camera_angle_x = transforms_json['camera_angle_x']
    camera_angle_y = transforms_json['camera_angle_y']


    transforms = transforms_json['frames']

    # Image ID
    n = 0

    ### Init Camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pf = pipeline.start(config) 
    device = pf.get_device()
    device.hardware_reset()

    logger = logging.getLogger("my_logger")
    logger.setLevel(logging.DEBUG)

    # 102 is left, 100 is right
    arm  = HagJkrc(logger, "192.168.10.102")
    arm.init_robot()
    print("Succesfully ini robot")

    cam = Camera(w, h, fps)
    print("Successfully init camera")

    valid_position_id = []
    for i in range(len(transforms)):
        file_path = transforms['frames'][i]['file_path']
        image_id = int(file_path.split('/')[-1].split('.')[0])
        valid_position_id.append(image_id)
    
    frames = []
    for i in valid_position_id:
        pnt = position_info[i]
        arm.move_to_point(pnt, 20)
        time.sleep(0.5)
        color_image, depth_image, colorizer_depth = cam.get_frame()
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        if len(str(i)) == 1:
            image_name = '000'+str(i)
        elif len(str(i)) == 2:
            image_name = '00'+str(i)
        elif len(str(i)) == 3:
            image_name = '0'+str(i)
        else:
            image_name =str(i)
        image_name += '.png'
        saved_img_name = os.path.join(SAVE_IMG_DIR,image_name)

        cv2.imwrite(saved_img_name,color_image)
        frame = {"file_path":SAVE_IMG_DIR,"sharpness":transforms[i]['sharpness'],"transform_matrix":transforms[i]['transform_matrix']}
        frames.append(frame)

        time.sleep(0.5)


    out = {
			"camera_angle_x": 1.2230789030277986,
			"camera_angle_y": 0.7517771630122048,
			"fl_x": 912.266,
			"fl_y": 911.672,
			"k1": 0,
			"k2": 0,
			"k3": 0,
			"k4": 0,
			"p1": 0,
			"p2": 0,
			"is_fisheye": False,
			"cx": 637.773,
			"cy": 375.817,
			"w": 1280,
			"h": 720,
			"aabb_scale": 8,
			"frames": frames,
		  }
    
    with open(output_json_path, "w") as outfile:
	    json.dump(out, outfile, indent=2)


if __name__ == '__main__':
    main()