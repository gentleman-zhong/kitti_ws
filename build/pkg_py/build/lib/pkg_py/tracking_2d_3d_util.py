import pandas as pd
import numpy as np
import cv2
from .kitti_util import Calibration


TRACKING_COLUMN_NAMES = ['frame', 'track_id', 'type', 'truncated', 'occluded', 'alpha', 'bbox_left', 'bbox_top','bbox_right', 'bbox_bottom', 'height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rot_y']
DETECTION_COLOR_DICT = {'Car':(255,255,0), 'Cyclist':(255,0,255), 'Pedestrian':(0,255,255)}

# 读取侦测框数据
def read_box_data(path):
    data = pd.read_csv(path ,header=None,sep = ' ')
    data.columns = TRACKING_COLUMN_NAMES
    data.loc[data.type.isin(['Truck','Van','Tram']),'type'] = 'Car'
    data = data[data.type.isin(['Car','Cyclist','Pedestrian'])]
    return data

# 画图片上2d盒子
def draw_2d_box(data,image, flag):
    type_array = np.array(data.loc[data['frame'] == flag, ['type']]) 
    box_array = np.array(data.loc[data['frame'] == flag, ['bbox_left','bbox_top','bbox_right','bbox_bottom']])
    # 有的图像可能不会出现车辆行人自行车啥的，这时候就没有tracking，自然没法处理
    if type_array.size > 0:                
        for typ,box in zip(type_array,box_array):
            top_left = int(box[0]),int(box[1])
            bottom_right = int(box[2]),int(box[3])
            cv2.rectangle(image, top_left, bottom_right,DETECTION_COLOR_DICT[typ[0]], 2)  #最后一个参数代表线的宽度

# 计算3d盒子顶点坐标
def compute_3d_box_cam2(h, w, l, x, y, z, yaw):
    R = np.array([[np.cos(yaw),0,np.sin(yaw)], [0,1,0], [-np.sin(yaw), 0, np.cos(yaw)]])
    x_corners = [l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2]
    y_corners = [0,0,0,0,-h,-h,-h,-h]
    z_corners = [w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2]
    corners_3d_cam2 = np.dot(R, np.vstack([x_corners,y_corners,z_corners]))
    corners_3d_cam2 += np.vstack([x,y,z])
    return corners_3d_cam2

# tarcking数据是相机坐标系,把它投影到点云坐标系
# 第一个参数是校正文件位置,第二个是相机坐标系下侦测框八个点
def compute_rect_to_velo(calib_filepath, corner_3d_cam2):
    calib = Calibration(calib_filepath,from_video=True)  # from_video代表会读取三个校正文件
    corner_3d_velo = calib.project_rect_to_velo(corner_3d_cam2.T).T
    return corner_3d_velo