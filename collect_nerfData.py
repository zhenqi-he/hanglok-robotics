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
from calibration.utils import *
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
    parser.add_argument('--calibrationData_dir', default=f'C://Users//HP//Desktop//hzq//hanglok-robotics//calibration//saved_07_06_16_38') 
    args = parser.parse_args()
 
    data_path = args.calibrationData_dir
    img_lists= glob.glob(os.path.join(data_path,'*.png'))
    position_txt_path = os.path.join(data_path,'position.txt')
    position_info = read_position(position_txt_path)
    # Camera Format, please refer to camera document
    fps, w, h = 30, 1280, 720

    # current time
    now = str(datetime.now())
    current_time = now[5:7]+'_'+now[8:10]+'_'+now[11:13]+'_'+now[14:16]
    # save path of images and position information
    SAVE_DIR = f'C://Users//HP//Desktop//hzq//hanglok-robotics//nerf//saved_'+current_time
    create_dir(SAVE_DIR)
    # SAVE_DIR = os.path.join(SAVE_DIR,current_time)
    # create_dir(SAVE_DIR)

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

    f = open(os.path.join(SAVE_DIR,'position.txt'),'w')
    
    
    for i in range(len(position_info)):
        pnt = position_info[i]
        arm.move_to_point(pnt, 20)
        time.sleep(1)
        color_image, depth_image, colorizer_depth = cam.get_frame()
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        saved_img_name = os.path.join(SAVE_DIR,str(i)+'_color.png')
        saved_dimg_name = os.path.join(SAVE_DIR,str(i)+'_depth.png')
        saved_cdimg_name = os.path.join(SAVE_DIR,str(i)+'_cdepth.png')
        cv2.imwrite(saved_img_name,color_image)
        cv2.imwrite(saved_dimg_name,depth_image)
        cv2.imwrite(saved_cdimg_name,colorizer_depth)
        time.sleep(5)


    
    
    f.close()


if __name__ == '__main__':
    main()