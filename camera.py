import time
import pyrealsense2 as rs
import numpy as np
import cv2

class Camera(object):
    '''
    realsense相机处理类
    '''
    def __init__(self, width=1280, height=720, fps=30):
        self.width = width
        self.height = height
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16,  fps)
        # self.align = rs.align(rs.stream.color) # depth2rgb
        self.pipeline.start(self.config)  # 开始连接相机
 
 
    def get_frame(self):
        frames = self.pipeline.wait_for_frames() # 获得frame (包括彩色，深度图)
        # 创建对齐对象
        align_to = rs.stream.color            # rs.align允许我们执行深度帧与其他帧的对齐
        align = rs.align(align_to)            # “align_to”是我们计划对齐深度帧的流类型。
        aligned_frames = align.process(frames)
        # 获取对齐的帧
        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame是对齐的深度图
        color_frame = aligned_frames.get_color_frame()
        colorizer = rs.colorizer()
        depthx_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        colorizer_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
        return color_image, depthx_image,colorizer_depth
    def get_extrinsics(self):
        frames = self.pipeline.wait_for_frames() # 获得frame (包括彩色，深度图)
        align_to = rs.stream.color            # rs.align允许我们执行深度帧与其他帧的对齐
        align = rs.align(align_to)            # “align_to”是我们计划对齐深度帧的流类型。
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame是对齐的深度图
        color_frame = aligned_frames.get_color_frame()

        # intr = color_frame.profile.as_video_stream_profile().intrinsics   #获取相机内参
        # profile1 = self.cfg.get_stream(rs.stream.color)
        # profile = self.cfg.get_stream(rs.stream.depth)
        extrinsics = color_frame.profile.as_video_stream_profile().extrinsics
        return extrinsics.rotation, extrinsics.translation
 
    def release(self):
        self.pipeline.stop()