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

'''
parameters:
INPUT_DATA_DIR: path for input images
CALIBRATION_DATA_DIR: path for previously taken images with auraco marker for calculating camera extrinsic matrix
                    images under this path are named as 1.png,2.png,...,n.png
'''

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--INPUT_DATA_DIR', default=r'C:\Users\HP\Desktop\hzq\hanglok-robotics\calibration\saved_07_06_16_38') 
    parser.add_argument('--DATA_TYPE', default='image') 
    parser.add_argument('--CALIBRATION_DATA_DIR', default=r'C:\Users\HP\Desktop\hzq\hanglok-robotics\calibration\saved_07_06_16_38')

    args = parser.parse_args()
    data_type = args.DATA_TYPE
    input_data_path = args.INPUT_DATA_DIR
    
    calibration_data_path = args.CALIBRATION_DATA_DIR

    input_image_path = os.path.join(input_data_path,'images')
    print('input_image_path ',input_image_path)
    create_dir(input_image_path)
    output_json_path = os.path.join(input_data_path,'transforms.json')
    
    # intri_matrix = get_cam_calibration()
    
    if data_type == 'image':
        calibration_images = glob.glob(os.path.join(calibration_data_path,'*png'))
        position_txt_path = os.path.join(calibration_data_path,'position.txt')
        position_info = read_position(position_txt_path)
        assert len(glob.glob(os.path.join(input_image_path,'*.png'))) == len(position_info), 'Wrong Input'+ str(len(position_info)) +'  '+str(len(glob.glob(os.path.join(input_image_path,'*.png'))))

        img_list = glob.glob(os.path.join(input_image_path,'*.png')) 
    elif data_type == 'video':
        
        video_path = os.path.join(calibration_data_path,'video.avi')
        print(video_path)
        video2images(video_path,input_image_path,f=10)

        img_list = glob.glob(os.path.join(input_image_path,'*.png')) 
    
    else:
        print("Wrong data type")
        exit()
    

    c2w_matrices_list = []
    camera_posX = []
    camera_posY = []
    camera_posZ = []

    # camera_Xaxis_start = []
    camera_Xaxis_end = []
    # camera_Yaxis_start = []
    camera_Yaxis_end = []
    # camera_Zaxis_start = []
    camera_Zaxis_end = []

    c_p_list = []
    frames = []

    transpose_matrix = np.array([[-0.02943494, -0.08614489,  0.9958477, -1.08057896 ],
                                 [ 0.02527841,  0.9958966 ,  0.08689629, -0.23705621],
                                 [-0.99924701,  0.02773124, -0.02713655, 0.01127695 ],
                                 [0          ,   0        ,  0         , 1]])
    # print(len(img_list))
    for i in range(len(img_list)):
        
        if len(str(i)) == 1:
            image_name = '000'+str(i)+'.png'
        elif len(str(i)) == 2:
            image_name = '00'+str(i)+'.png'
        elif len(str(i)) == 3:
            image_name = '0'+str(i)+'.png'
        
        # img_path = os.path.join(input_image_path,image_name)
        img_path = os.path.join(input_image_path,str(i)+'.png')

        calibrate_image_name = os.path.join(calibration_data_path,str(i)+'.png')
        # status,R_target2camera,T_target2camera = cal_camera_extrinsic(calibrate_image_name,intr_matrix=intri_matrix)
        if data_type == 'image':
            status,R_target2camera,T_target2camera = cal_camera_extrinsic(calibrate_image_name)
        else:
            status,R_target2camera,T_target2camera = cal_camera_extrinsic(img_path)
        if status:
            matrix_w2c = np.row_stack((np.column_stack((R_target2camera,T_target2camera)),np.array([0,0,0,1])))
            R_c2w = np.linalg.inv(R_target2camera)
            T_c2w = (-1)*R_c2w@T_target2camera
            # camera_p = matrix_w2c@np.array([[0],[0],[0],[1]])
            
            matrix_c2w =  np.linalg.inv(matrix_w2c)
            # matrix_c2w = np.row_stack((np.column_stack((R_c2w,T_c2w)),np.array([0,0,0,1])))
            transf = np.array([
            [1,0,0,0],
            [0,-1,0,0],
            [0,0,-1,0],
            [0,0,0,1.],
            ])
            transformer_matrix_c2w = matrix_c2w@transf
            c2w_matrices_list.append(matrix_c2w)
            frame = {"file_path":img_path,"sharpness":50,"transform_matrix":transformer_matrix_c2w.tolist()}
            frames.append(frame)

            camera_p =  transformer_matrix_c2w@np.array([0,0,0,1])
            
            # camera_posX.append(camera_p[0])
            # camera_posY.append(camera_p[1])
            # camera_posZ.append(camera_p[2])
            c_p_list.append(camera_p)

            camera_Xaxis_end.append(transformer_matrix_c2w@np.array([1,0,0,1]))
            camera_Yaxis_end.append(transformer_matrix_c2w@np.array([0,1,0,1]))
            camera_Zaxis_end.append(transformer_matrix_c2w@np.array([0,0,1,1]))

            


            
            # camera_pos.append(camera_p)
    # print(len(frames))
    #frame = {"file_path":name,"sharpness":b,"transform_matrix": c2w}
    out = {
			"camera_angle_x": 1.2235306391055296720,
			"camera_angle_y": 0.7521676402860731,
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
    # print(c_p_list)

    camera_posX = [i[0]/i[3] for i in c_p_list]
    camera_posY = [i[1]/i[3] for i in c_p_list]
    camera_posZ = [i[2]/i[3] for i in c_p_list]

    camera_Xaxis_end_pntX = [i[0]/i[3] for i in camera_Xaxis_end]
    camera_Xaxis_end_pntY = [i[1]/i[3] for i in camera_Xaxis_end]
    camera_Xaxis_end_pntZ = [i[2]/i[3] for i in camera_Xaxis_end]

    camera_Yaxis_end_pntX = [i[0]/i[3] for i in camera_Yaxis_end]
    camera_Yaxis_end_pntY = [i[1]/i[3] for i in camera_Yaxis_end]
    camera_Yaxis_end_pntZ = [i[2]/i[3] for i in camera_Yaxis_end]

    camera_Zaxis_end_pntX = [i[0]/i[3] for i in camera_Zaxis_end]
    camera_Zaxis_end_pntY = [i[1]/i[3] for i in camera_Zaxis_end]
    camera_Zaxis_end_pntZ = [i[2]/i[3] for i in camera_Zaxis_end]

    

    
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.scatter(camera_posX, camera_posY, camera_posZ, color='green')
    # ax.scatter(gripper_posX, gripper_posY, gripper_posZ, 'green')
    ax.scatter([0], [0], [0], color='black')
    # for i in range(len(camera_Yaxis_end)):
    #     ax.plot([camera_posX[i], camera_Xaxis_end_pntX[i]], [camera_posY[i],camera_Xaxis_end_pntY[i]],zs=[camera_posZ[i],camera_Xaxis_end_pntZ[i]],color='red')
    #     ax.plot([camera_posX[i], camera_Yaxis_end_pntX[i]], [camera_posY[i],camera_Yaxis_end_pntY[i]],zs=[camera_posZ[i],camera_Yaxis_end_pntZ[i]],color='green')
    #     ax.plot([camera_posX[i], camera_Zaxis_end_pntX[i]], [camera_posY[i],camera_Zaxis_end_pntY[i]],zs=[camera_posZ[i],camera_Zaxis_end_pntZ[i]],color='blue')
    plt.show()
        
if __name__ == '__main__':
    main()

