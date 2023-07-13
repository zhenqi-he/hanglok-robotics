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