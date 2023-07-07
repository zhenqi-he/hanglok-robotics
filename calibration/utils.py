import os
import numpy as np
import math
import cv2.aruco as aruco
import cv2
import json

def create_dir(dir):
    if not os.path.exists(dir):
        os.mkdir(dir)

def read_position(dir):
    positions = []
    with open(dir,'r') as f:
        lines = f.readlines()
        # print(len(lines))
        for line in lines:
            pos = line.split(',')
            pos = [float(i) for i in pos]
            positions.append(pos)
    return positions


def get_cam_calibration(cam_cal_data_path):
    # load camera calibration data to get camera intrinsic matrix
    cali_data = json.load(open(cam_cal_data_path))
    intrinsic_left = np.zeros((3,3))
    intrinsic_right = np.zeros((3,3))
    axis_lists =['x','y','z']
    for i in range(3):
        for j in range(3):
            intrinsic_left[i,j] = float(cali_data['intrinsic_left.'+axis_lists[i]+'.'+axis_lists[j]])
            intrinsic_right[i,j] = float(cali_data['intrinsic_right.'+axis_lists[i]+'.'+axis_lists[j]])
    # get intrinsic camera matrices for both left and right cameras and take average
    return (intrinsic_left+intrinsic_right)/2


def angle2rotation(rx, ry, rz):
    # input : row, yaw , pitch
    # output: corresponding rotation matrix
    Rx = np.array([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]])
    Ry = np.array([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]])
    Rz = np.array([[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])
    R = Rz @ Ry @ Rx
    return R

def gripper2base(pnt):
    x, y, z, rx, ry, rz = pnt
    # convert the rx,ry,rz
    # thetaX = rx / 180 * math.pi
    # thetaY = ry / 180 * math.pi
    # thetaZ = rz / 180 * math.pi
    # Calculate the rotation matrix from gripper to base, the shapo is 3x3
    # R_gripper2base = angle2rotation(thetaX, thetaY, thetaZ)
    R_gripper2base = angle2rotation(rx, ry, rz)
    # Calculate the transform matrix from gripper to base (base is (0,0,0)), the shape is 3x1
    T_gripper2base = np.array([[x], [y], [z]])
    # print(T_gripper2base)

    return R_gripper2base, T_gripper2base
    # return np.array([[rx],[ry],[rz]]), T_gripper2base

def target2camera(img_path,aruco_dict,intr_matrix):
    ## the distortion coefficients of our camera is zero
    distCoeffs = np.zeros((5,1))

    img = cv2.imread(img_path)
    status = False
    R_target2camera = None
    rvec = None
    T_target2camera = None
    parameters =  aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(img)

    world_coor = np.zeros((4,3),dtype=np.float64)
    world_coor[0,:] = [500,500,0]
    world_coor[1,:] = [500,-500,0]
    world_coor[2,:] = [-500,-500,0]
    world_coor[3,:] = [-500,500,0]
    # world_coor[0,:] = [1000,1000,0]
    # world_coor[1,:] = [1000,0,0]
    # world_coor[2,:] = [0,0,0]
    # world_coor[3,:] = [0,1000,0]
    # world_coor[0,:] = [-500,500,0]
    # world_coor[1,:] = [500,500,0]
    # world_coor[2,:] = [500,-500,0]
    # world_coor[3,:] = [-500,-500,0]
 
    # if successfully detected 4 corner points
    if markerIds is not None:
        if markerCorners[0][0].shape[0] == 4:
            
            retval, rvec, T_target2camera = cv2.solvePnP(world_coor, markerCorners[0][0], intr_matrix, distCoeffs,flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if retval:
                R_target2camera = angle2rotation(rvec[0][0],rvec[1][0],rvec[2][0])
                status = True
                # print(rvec)
    return status,R_target2camera,T_target2camera
    # return status,rvec,T_target2camera




