import os
import shutil
import glob
from utils import *
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--DATA_PATH', default=f'C://Users//HP//Desktop//hzq//hanglok-robotics//nerf//saved_07_06_18_40') 
    parser.add_argument('--TAR_PATH', default=f'C://Users//HP//Desktop//hzq//nerf_data//processed_07_06_18_40_cal_cam')
    # parser.add_argument('--calibrationData_dir', default=f'C://Users//HP//Desktop//hzq//hanglok-robotics//calibration//saved_07_06_14_12/') # images are saved under this dir as n.png and gripper position information is saved in position.txt
    args = parser.parse_args()

    DATA_PATH = args.DATA_PATH
    TAR_PATH = args.TAR_PATH
    print(DATA_PATH)

    create_dir(TAR_PATH)
    create_dir(os.path.join(TAR_PATH,'images'))

    img_lists = sorted(glob.glob(os.path.join(DATA_PATH,'*.png')))

    # print(img_lists[:5])
    for imgP in img_lists:
        # print(imgP)
        if 'color' in imgP:
            image_name = imgP[:-10].replace(DATA_PATH+'/','')
            print(image_name)
            if len(image_name) == 1:
                image_name = '000'+image_name
            elif len(image_name) == 2:
                image_name = '00'+image_name
            elif len(image_name) == 3:
                image_name = '0'+image_name
            # elif len(image_name) == 4:
            #     image_name = image_name
            image_name += '.png'
            
            # print(image_name)
            des_path = os.path.join(TAR_PATH,'images',image_name)
            # print(des_path)
            shutil.copy2(imgP,des_path)
    print(des_path)


if __name__ == '__main__':
    main()


        