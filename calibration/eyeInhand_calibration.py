import os
import numpy as np
import math
import cv2.aruco as aruco
import cv2
import json
from utils import *
import argparse
import glob
import logging
from datetime import datetime
import random
# we use the 4x4 sized at 1000x1000 aruco code
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cameraData', default=f'C://Users//HP//Desktop//hzq//hanglok-robotics//calibration//calibration_data.json') 
    # parser.add_argument('--calibrationData_dir', default=f'C://Users//HP//Desktop//hzq//hanglok-robotics//calibration//saved')
    parser.add_argument('--calibrationData_dir', default=f'C://Users//HP//Desktop//hzq//hanglok-robotics//calibration//saved_07_21_12_04/')
    # parser.add_argument('--calibrationData_dir', default=f'C://Users//HP//Desktop//hzq//hanglok-robotics//calibration//saved_07_06_14_12/') # images are saved under this dir as n.png and gripper position information is saved in position.txt
    args = parser.parse_args()

    cam_cal_data_path = args.cameraData
    data_path = args.calibrationData_dir
    img_lists= glob.glob(os.path.join(data_path,'*.png'))
    position_txt_path = os.path.join(data_path,'position.txt')
    position_info = read_position(position_txt_path)
    # camera_intrinsic_matrix = get_cam_calibration(cam_cal_data_path)
    camera_intrinsic_matrix = np.array([[912.266, 0, 637.773],
                                        [0, 911.672, 375.817],
                                        [0,       0,       1]], dtype = "double")


    # check data validity
    assert len(img_lists)==len(position_info) ,'Wrong input data, number of images does not match the number of gripper data'

    # create log 
    now = datetime.now()
    create_dir(os.path.join(data_path,'log'))
    n = len(glob.glob(os.path.join(data_path,'log','*.txt')))
    logging.basicConfig(filename=os.path.join(data_path,'log','log_{}.txt'.format(n)), level=logging.INFO,
                            format='[%(asctime)s.%(msecs)03d] %(message)s', datefmt='%H:%M:%S')
    logging.info('camera data path : {}'.format(cam_cal_data_path))
    logging.info('images and gripper position data path : {}'.format(data_path))
    logging.info('Taltal {} points detected in gripper position document'.format(len(position_info)))

    R_gripper2base_lists = []
    T_gripper2base_lists = []
    R_base2gripper_lists = []
    T_base2gripper_lists = []
    R_target2camera_lists = []
    T_target2camera_lists = []

    valid_image_id = []
    for i in range(len(position_info)):
        img_path = os.path.join(data_path,str(i)+'.png')
        status,R_target2camera,T_target2camera = target2camera(img_path,aruco_dict,camera_intrinsic_matrix)
        # print(status)
        
        if status == True:
            R_target2camera_lists.append(R_target2camera)
            T_target2camera_lists.append(T_target2camera)

            R_gripper2base, T_gripper2base = gripper2base(position_info[i])
            R_gripper2base_lists.append(R_gripper2base)
            T_gripper2base_lists.append(T_gripper2base)

            M_gripper2base = np.row_stack((np.column_stack((R_gripper2base,T_gripper2base)),np.array([0,0,0,1])))
            M_base2gripper = np.linalg.inv(M_gripper2base)
            R_base2gripper = M_base2gripper[:3,:3]
            T_base2gripper = M_base2gripper[:3,3]
            R_base2gripper_lists.append(R_base2gripper)
            T_base2gripper_lists.append(T_base2gripper)
            valid_image_id.append(i)

        else:
            # if not successfully detected the 4 corners, then ignore this data
            continue
    
    logging.info('The number of valid data : {}'.format(len(R_gripper2base_lists)))
    print('The number of valid data : {}'.format(len(R_gripper2base_lists)))

    # Use 80% of data to calculate the transformation and rotation matrix, and the rest 20% is used for checking
    num_train = int(len(R_gripper2base_lists)*0.8)
    # num_train = len(R_gripper2base_lists)-5

    ## gripper to base
    print("Num of training images: ",num_train)
    logging.info("Valid images Id: {}".format(valid_image_id))
    logging.info("Num of training images: {}".format(num_train))
    print(valid_image_id)
    R_gripper2base_lists_train = R_gripper2base_lists[:num_train]
    T_gripper2base_lists_train = T_gripper2base_lists[:num_train]
    ### base to gripper
    R_base2gripper_lists_train = R_base2gripper_lists[:num_train]
    T_base2gripper_lists_train = T_base2gripper_lists[:num_train]

    R_target2camera_lists_train = R_target2camera_lists[:num_train]
    T_target2camera_lists_train = T_target2camera_lists[:num_train]

    R_camera2gripper, T_camera2gripper = cv2.calibrateHandEye(R_gripper2base_lists_train, T_gripper2base_lists_train,R_target2camera_lists_train, T_target2camera_lists_train)
    # R_base2target, T_base2target,R_gripper2cam, T_gripper2cam = cv2.calibrateRobotWorldHandEye(R_target2camera_lists_train, T_target2camera_lists_train,R_base2gripper_lists_train, T_base2gripper_lists_train,method=cv2.CALIB_HAND_EYE_ANDREFF)
    # print(R_camera2gripper)
    # print(T_camera2gripper)

    # logging.info('R camera to gripper: {}'.format(R_camera2gripper))
    # logging.info('T camera to gripper: {}'.format(T_camera2gripper))
    # save the calculated matrix to a json file
    camera2gripper_dict = {}
    for i in range(3):
        camera2gripper_dict['T_'+str(i+1)] = T_camera2gripper[i][0]
        for j in range(3):
            camera2gripper_dict['R'+'_'+str(i+1)+str(j+1)] =  R_camera2gripper[i][j]
    
    with open(os.path.join('camera2base.json'),'w') as f:
        json.dump(camera2gripper_dict, f, ensure_ascii=False,indent=2)
    

    ### checking:
    # target2base should remain constant during the whole calibration, thus gripper2base*camera2gripper*target2camera should maintain the same for different gripper positions
    
    R_gripper2base_lists_test = R_gripper2base_lists[num_train:]
    T_gripper2base_lists_test = T_gripper2base_lists[num_train:]
    R_target2camera_lists_test = R_target2camera_lists[num_train:]
    T_target2camera_lists_test = T_target2camera_lists[num_train:]
    # R_gripper2base_lists_test = R_gripper2base_lists[:num_train]
    # T_gripper2base_lists_test = T_gripper2base_lists[:num_train]
    # R_base2gripper_lists_test = R_base2gripper_lists[:num_train]
    # T_base2gripper_lists_test = T_base2gripper_lists[:num_train]
    # R_target2camera_lists_test = R_target2camera_lists[:num_train]
    # T_target2camera_lists_test = T_target2camera_lists[:num_train]
    
    M_target2base_list = []
    M_target2base_diff_list = []
    l2_distance_list = []
    for i in range(len(R_gripper2base_lists_test)):
        M_target2camera = np.row_stack((np.column_stack((R_target2camera_lists_test[i],T_target2camera_lists_test[i])),np.array([0,0,0,1])))
        M_camera2gripper = np.row_stack((np.column_stack((R_camera2gripper,T_camera2gripper)),np.array([0,0,0,1])))
        
        M_gripper2base = np.row_stack((np.column_stack((R_gripper2base_lists_test[i],T_gripper2base_lists_test[i])),np.array([0,0,0,1])))
        # M_target2camera = np.row_stack((np.column_stack((cv2.Rodrigues(R_target2camera_lists_test[i])[0],T_target2camera_lists_test[i])),np.array([0,0,0,1])))
        # M_camera2gripper = np.row_stack((np.column_stack((R_camera2gripper,T_camera2gripper)),np.array([0,0,0,1])))
        # M_gripper2base = np.row_stack((np.column_stack((cv2.Rodrigues(R_gripper2base_lists_test[i])[0],T_gripper2base_lists_test[i])),np.array([0,0,0,1])))
        M_target2base = M_gripper2base@M_camera2gripper@M_target2camera
        
        M_target2base_list.append(M_target2base)

        # print(M_target2base)
        # print('\n')

        # if i >= 1:
        #     l2 = np.sum((M_target2base_list[i-1]-M_target2base)**2)
        #     l2_distance_list.append(l2)
        #     print(M_target2base)
        #     M_target2base_diff_list.append(M_target2base_list[i-1]-M_target2base)
    
    for i in range(len(M_target2base_list)):
        for j in range(i+1,len(M_target2base_list)):
            l2 = np.sum((M_target2base_list[i]-M_target2base_list[j])**2)
            l2_distance_list.append(l2)
    # print(l2_distance_list)
    # print(sum(l2_distance_list)/len(l2_distance_list))
    # print(M_target2base_list)
    # print(M_target2base_diff_list)

    test_coor = np.array([0,0,0,1])
    test_coor_inBase_lists = []

    for i in range(len(M_target2base_list)):
        test_coor = np.array([0,0,0,1])
        test_coor_inBase = M_target2base_list[i]@test_coor
        print(test_coor_inBase,'\n')
        # print(test_coor_inBase)
        test_coor_inBase_lists.append(test_coor_inBase)
    
    # print(test_coor_inBase_lists)



if __name__ == '__main__':
    main()



    

   
