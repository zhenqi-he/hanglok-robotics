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

### This is collecting data for hand-eye calibration
jaka_ip = "192.168.10.200"
def main():
    # current time
    now = str(datetime.now())
    current_time = now[5:7]+'_'+now[8:10]+'_'+now[11:13]+'_'+now[14:16]
    # save path of images and position information
    SAVE_DIR = f'C://Users//HP//Desktop//hzq//hanglok-robotics//calibration//saved_'+current_time

    fps, w, h = 30, 1280, 720

    # current time
    now = str(datetime.now())
    current_time = now[5:7]+'_'+now[8:10]+'_'+now[11:13]+'_'+now[14:16]
    video_name = os.path.join(SAVE_DIR,'video.avi')
    # save path of images and position information
    

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
    arm  = HagJkrc(logger, jaka_ip)
    arm.init_robot()
    print("Succesfully ini robot")



    cam = Camera(w, h, fps)
    print("Successfully init camera")
    
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(video_name, fourcc, fps, (w, h), True)
    
    while True:
        
        image, depth_image, colorizer_depth = cam.get_frame()
        
        cv2.imshow('frame', image)
        out.write(image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    out.release()
    cam.release()


if __name__ == '__main__':
    main()
