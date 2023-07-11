import os
import numpy as np
import math
import cv2
import json
from utils import *
import argparse
import glob
import logging
from datetime import datetime
import random
import glob
import matplotlib.pyplot as plt

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--INPUT_DATA_DIR', default=f'C://Users//HP//Desktop//hzq//nerf_data//processed_07_06_18_40_cal_cam') 
    parser.add_argument('--CALIBRATION_DATA_DIR', default=f'C://Users//HP//Desktop//hzq//hanglok-robotics//calibration//saved_07_06_16_38/')
    parser.add_argument('--COLMAP_JSON_DIR', default=f'C://Users//HP//Desktop//hzq//hanglok-robotics//calibration//saved_07_06_16_38/')

    args = parser.parse_args()
    input_data_path = args.INPUT_DATA_DIR
    calibration_data_path = args.CALIBRATION_DATA_DIR
    COLMAP_JSON_DIR = args.COLMAP_JSON_DIR

    input_image_path = os.path.join(input_data_path,'images')
    output_json_path_vison_method = os.path.join(input_data_path,'points_vision_method.json')
    output_json_path_COLMAP = os.path.join(input_data_path,'points_COLMAP.json')
    position_txt_path = os.path.join(calibration_data_path,'position.txt')
    position_info = read_position(position_txt_path)
    transforms_frames = read_transformsJSON_data(COLMAP_JSON_DIR)
    COLMAP_file_names = [i['file_path'][-8:] for i in transforms_frames]

    assert len(glob.glob(os.path.join(input_image_path,'*.png'))) == len(position_info), 'Wrong Input'+ str(len(position_info)) +'  '+str(len(glob.glob(os.path.join(input_image_path,'*.png'))))

    c2w_matrices_list = []
    camera_posX = []
    camera_posY = []
    camera_posZ = []

    COLMAP_camera_posX = []
    COLMAP_camera_posY = []
    COLMAP_camera_posZ = []

    for i in range(len(position_info)):
        
        if len(str(i)) == 1:
            image_name = '000'+str(i)+'.png'
        elif len(str(i)) == 2:
            image_name = '00'+str(i)+'.png'
        elif len(str(i)) == 3:
            image_name = '0'+str(i)+'.png'
        else:
            continue
        
        if image_name not in COLMAP_file_names:
            continue
        else:
            COLMAP_c2w_matrix = np.array(transforms_frames[COLMAP_file_names.index(image_name)]['transform_matrix'])
            COLMAP_camera_p = COLMAP_c2w_matrix@np.array([[0],[0],[0],[1]])
            COLMAP_camera_posX.append(COLMAP_camera_p[0])
            COLMAP_camera_posY.append(COLMAP_camera_p[1])
            COLMAP_camera_posZ.append(COLMAP_camera_p[2])
        img_path = os.path.join(input_image_path,image_name)
        calibrate_image_name = os.path.join(calibration_data_path,str(i)+'.png')
        status,R_target2camera,T_target2camera = cal_camera_extrinsic(calibrate_image_name)
        if status:
            matrix_c2w = np.row_stack((np.column_stack((R_target2camera,T_target2camera)),np.array([0,0,0,1])))
            matrix_c2w =  np.linalg.inv(matrix_c2w)
            camera_p = matrix_c2w@np.array([[0],[0],[0],[1]])
            camera_posX.append(camera_p[0])
            camera_posY.append(camera_p[1])
            camera_posZ.append(camera_p[2])
            
            c2w_matrices_list.append(matrix_c2w)
    
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.scatter(camera_posX, camera_posY, camera_posZ, 'green')
    ax.scatter(COLMAP_camera_posX, COLMAP_camera_posY, COLMAP_camera_posZ, 'red')
    plt.show()


