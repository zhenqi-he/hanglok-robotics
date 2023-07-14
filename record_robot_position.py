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


jaka_ip = "10.5.5.100"
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


    f = open(os.path.join(SAVE_DIR,'position.txt'),'w')
    
    
    start_pnt = arm.read_actual_tcp_point_all()
    str_pnt = ''
    for i in range(6):
        if i == 0:
            str_pnt += str(start_pnt[i])
        else:
            str_pnt += ','+str(start_pnt[i])
    f.write(str_pnt+'\n')
    print("Robot init position {}".format(start_pnt))

    cam = Camera(w, h, fps)
    print("Successfully init camera")
    image, depth_image, colorizer_depth = cam.get_frame()
    saved_img_name = os.path.join(SAVE_DIR,str(n)+'.png')
    cv2.imwrite(saved_img_name,image)
    n += 1

    print("Please input 1 to keep record or input 0 to stop")
    decision = int(input())
    while decision:
        pnt = arm.read_actual_tcp_point_all()

        # Check whether user moves the robot, if not, wait for 30 seconds
        while pnt == start_pnt:
            print("Please change robot position")
            time.sleep(30)
            pnt = arm.read_actual_tcp_point_all()
            # print("start_pnt: ",start_pnt)
            # print("start_pnt: ",pnt)

        str_pnt = ''
        for i in range(6):
            if i == 0:
                str_pnt += str(pnt[i])
            else:
                str_pnt += ','+str(pnt[i])
        f.write(str_pnt+'\n')
        print("Robot init position {}".format(pnt))


        image, depth_image, colorizer_depth = cam.get_frame()
        saved_img_name = os.path.join(SAVE_DIR,str(n)+'.png')
        cv2.imwrite(saved_img_name,image)
        n += 1

        start_pnt = pnt
        print("Please input 1 to keep record or input 0 to stop")
        decision = int(input())
    f.close()


if __name__ == '__main__':
    main()
