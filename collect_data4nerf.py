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

# def record_pnt2df(df,pnt):
#     assert len(pnt)==6, "Point format Wrong"
#     # d = {}
#     n = len(df['timestep'].to_list())
#     print('n: ',n)
#     for i in range(6):
#         df.loc[n-1,'pos_'+str(i)] = pnt[i]
#         # d['pos_'+str(i)] = pnt[i]
#         # print(pnt[i])
#     # df.concat(d)


def create_dir(dir):
    if not os.path.exists(dir):
        os.mkdir(dir)

def main():
    # Camera Format, please refer to camera document
    fps, w, h = 30, 1280, 720
    # step setting for taking images at different position
    step = 30
    step_length = 5
    # current time
    now = str(datetime.now())
    current_time = now[5:7]+'_'+now[8:10]+'_'+now[11:13]+'_'+now[14:16]
    # save path of images and position information
    SAVE_DIR = f'C://Users//gzkxy//Desktop//zqh//saved//'
    SAVE_DIR = os.path.join(SAVE_DIR,current_time)
    create_dir(SAVE_DIR)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pf = pipeline.start(config) 
    device = pf.get_device()
    device.hardware_reset()

    # Init Camera
    

    logger = logging.getLogger("my_logger")
    logger.setLevel(logging.DEBUG)

    # 200 is left and 100 is right
    arm  = HagJkrc(logger, "192.168.15.200")
    arm.init_robot()
    print("Succesfully ini robot")

    # Init a df to record time and position infomation
    # df = pd.DataFrame({'timestep':[],'pos_0':[],"pos_1":[],'pos_2':[],"pos_3":[],'pos_4':[],"pos_5":[]})
    f = open(os.path.join(SAVE_DIR,'position.txt'),'w')
    # df = pd.DataFrame({'pos_0':[],"pos_1":[],'pos_2':[],"pos_3":[],'pos_4':[],"pos_5":[]})
    # Start Point
    # df.loc[0,'timestep'] = 0
    start_pnt = arm.read_actual_tcp_point_all()
    str_pnt = ''
    for i in range(6):
        str_pnt += ','+str(start_pnt[i])
    f.write(str_pnt+'\n')
    print("Robot init position {}".format(start_pnt))
    # Record Start Point
    
    # record_pnt2df(df,start_pnt)
    print('ok')

    cam = Camera(w, h, fps)
    print("Successfully init camera")
    color_image, depth_image, colorizer_depth = cam.get_frame()
    

    # images = np.hstack((color_image, depth_colormap))  
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    
    saved_img_name = os.path.join(SAVE_DIR,str(0)+'_color.png')
    saved_dimg_name = os.path.join(SAVE_DIR,str(0)+'_depth.png')
    saved_cdimg_name = os.path.join(SAVE_DIR,str(0)+'_cdepth.png')
    # print(color_image.shape)
    cv2.imwrite(saved_img_name,color_image)
    cv2.imwrite(saved_dimg_name,depth_image)
    cv2.imwrite(saved_cdimg_name,colorizer_depth)


    # Move Jaka to different positions to record rbgd data
    # 6 directions (forward/backward for xyz)
    n = 0
    for i in range(6):
        
        axis = i//2 # 0 if for moving paralle to the windows, 1 is for moving vertical to the windows
        direction = i%2 # check whether go forward or backward, odd for forward, even for backward
        for s in range(step):
            print("current axis : {}, current step: {}".format(axis,s))
            current_pnt = arm.read_actual_tcp_point_all()
            # n = len(df['timestep'].to_list())
            # df.loc[n,'timestep'] = n
            # record_pnt2df(df,current_pnt)

            if direction:
                current_pnt[axis] += step_length
            else:
                current_pnt[axis] -= step_length
            str_pnt = ''    
            for i in range(6):
                str_pnt += ','+str(current_pnt[i])
            f.write(str_pnt+'\n')
            arm.move_to_point(current_pnt, 10)
            # sleep 1 second
            time.sleep(1)
            # get image from depth camera
            color_image, depth_image, colorizer_depth = cam.get_frame()
            n += 1
            saved_img_name = os.path.join(SAVE_DIR,str(n)+'_color.png')
            saved_dimg_name = os.path.join(SAVE_DIR,str(n)+'_depth.png')
            saved_cdimg_name = os.path.join(SAVE_DIR,str(n)+'_cdepth.png')
            cv2.imwrite(saved_img_name,color_image)
            cv2.imwrite(saved_dimg_name,depth_image)
            cv2.imwrite(saved_cdimg_name,colorizer_depth)
#             time.sleep(1)
        # go back to init position for next direction moving
        arm.move_to_point(start_pnt, 10)
    cam.release()
    # df.to_csv(os.path.join(SAVE_DIR,'positions.csv'),index=False)
    f.close()
if __name__ == '__main__':
    main()

    
    