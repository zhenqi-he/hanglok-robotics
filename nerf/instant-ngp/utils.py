import os
import numpy as np
import math
import cv2.aruco as aruco
import cv2
import json

def create_dir(dir):
    if not os.path.exists(dir):
        print(dir)
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

def angle2rotation(rx, ry, rz):
    # input : row, yaw , pitch
    # output: corresponding rotation matrix
    Rx = np.array([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]])
    Ry = np.array([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]])
    Rz = np.array([[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])
    R = Rz @ Ry @ Rx
    return R
def get_cam_calibration(cam_cal_data_path=f'C://Users//HP//Desktop//hzq//hanglok-robotics//calibration//calibration_data.json'):
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


def read_transformsJSON_data(dir):
    transforms = json.load(open(dir))['frames']
    return transforms

def video2images(video_path,output_path, f=10): #images will be saved in the forlder named images in the same directory of video path, f = image per f frames
   
    videoCapture = cv2.VideoCapture(video_path)
    success, frame = videoCapture.read()
    print(success)
    if success:
        fps = videoCapture.get(cv2.CAP_PROP_FPS)
        tf = videoCapture.get(cv2.CAP_PROP_FRAME_COUNT)
        
        duration = max(int(tf/fps),0)
        n = 0
        for i in range(1,int(tf),f):
            
            try:
                videoCapture.set(cv2.CAP_PROP_POS_FRAMES, i)
                rval,frame = videoCapture.read()  
                cv2.imwrite(os.path.join(output_path,str(n)+'.png'), frame) 
                n += 1
            except:
                videoCapture.set(cv2.CAP_PROP_POS_FRAMES, i+1)
                rval,frame = videoCapture.read()  
                cv2.imwrite(os.path.join(output_path,str(n)+'.png'), frame)
                n += 1


def check_calculated_extrinsicM(corners,w2c,intr_matrix):
    l2_distance = 0.0
    
    for n,p in enumerate([[0,1,0],[1,1,0],[1,0,0],[0,0,0]]):
        p.append(1)
        camera_coor_pnt = w2c@np.array(p)
        # print(camera_coor_pnt.shape)
        camera_coor_pnt_3 = np.array([camera_coor_pnt[0]/camera_coor_pnt[3],camera_coor_pnt[1]/camera_coor_pnt[3],camera_coor_pnt[2]/camera_coor_pnt[3]])
        pix_coor = intr_matrix@camera_coor_pnt_3
        # print(pix_coor.shape)
        pix_cor_2D = [pix_coor[0]/pix_coor[2],pix_coor[1]/pix_coor[2]]
        l2_distance += 0.5*(math.pow((corners[n][0]-pix_cor_2D[0]),2)+math.pow((corners[n][1]-pix_cor_2D[1]),2))
        # print("gt pixel coordinate: ({},{})\npredicted pixel coordinate: ({},{})".format(corners[n][0],corners[n][1],pix_cor_2D[0],pix_cor_2D[1]))
    l2_distance = l2_distance/corners.shape[0]
    # print('mean l2 distance ', l2_distance)
    if l2_distance > 0.1:
        return False
    else:
        return True
    
def cal_camera_extrinsic(img_path,aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_4X4_1000),intr_matrix= np.array([[912.266, 0, 637.773],[0, 911.672, 375.817],[0,0,1]], dtype = "double")):
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

    world_coor = np.zeros((4,3),dtype=np.float32)
    world_coor[0,:] = [0,1,0]
    world_coor[1,:] = [1,1,0]
    world_coor[2,:] = [1,0,0]
    world_coor[3,:] = [0,0,0]
    # world_coor[0,:] = [0,1,0]
    # world_coor[1,:] = [1,1,0]
    # world_coor[2,:] = [1,0,0]
    # world_coor[3,:] = [0,0,0]
    # world_coor[0,:] = [-0.5,0.5,0]
    # world_coor[1,:] = [0.5,0.5,0]
    # world_coor[2,:] = [0.5,-0.5,0]
    # world_coor[3,:] = [-0.5,-0.5,0]
    

    
    # if successfully detected 4 corner points
    if markerIds is not None:
        if markerCorners[0][0].shape[0] == 4:
            # corners = np.zeros((4,2))
            # corners[0,:] = markerCorners[0][0][0]
            # corners[1,:] = markerCorners[0][0][1]
            # corners[2,:] = markerCorners[0][0][2]
            # corners[3,:] = markerCorners[0][0][3]
            # markerCorners = markerCorners[0][0].transpose(3,0,1,2)
            # retval, rvec, T_target2camera = cv2.solvePnP(world_coor, markerCorners[0][0], intr_matrix, distCoeffs,flags=cv2.SOLVEPNP_IPPE_SQUARE)
            retval, rvec, T_target2camera = cv2.solvePnP(world_coor, markerCorners[0][0], intr_matrix, distCoeffs )
            # print(corners)
            # retval, mtx, dist, rvec, T_target2camera = cv2.solvePnP(objectPoints=[np.float32(world_coor)], imagePoints=[np.float32(corners)], imageSize=(1280,720),cameraMatrix=intr_matrix, distCoeffs=distCoeffs )
            # print(rvec[0][0])
            if retval:
                R_target2camera = cv2.Rodrigues(rvec)[0]
                T_target2camera = T_target2camera
                matrix_w2c = np.row_stack((np.column_stack((R_target2camera,T_target2camera)),np.array([0,0,0,1])))
                if check_calculated_extrinsicM(markerCorners[0][0],matrix_w2c, intr_matrix):
                    status = True
                # print(rvec)
    return status,R_target2camera,T_target2camera


