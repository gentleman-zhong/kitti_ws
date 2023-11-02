import pandas as pd
import numpy as np
import cv2
from .kitti_util import Calibration
from collections import deque


TRACKING_COLUMN_NAMES = ['frame', 'track_id', 'type', 'truncated', 'occluded', 'alpha', 'bbox_left', 'bbox_top','bbox_right', 'bbox_bottom', 'height', 'width', 'length', 'pos_x', 'pos_y', 'pos_z', 'rot_y']
DETECTION_COLOR_DICT = {'Car':(255,255,0), 'Cyclist':(255,0,255), 'Pedestrian':(0,255,255)}
IMU_COLUMN_NAMES = ['lat', 'lon', 'alt', 'roll', 'pitch', 'yaw', 'vn', 've', 'vf', 'vl', 'vu', 'ax', 'ay', 'az', 'af','al', 'au', 'wx', 'wy', 'wz', 'wf', 'wl', 'wu', 'posacc', 'velacc', 'navstat', 'numsats', 'posmode','velmode', 'orimode']

class Object():
    def __init__(self):
        self.locations = deque(maxlen=20)

    def update(self, displacement, yaw_change):
        for i in range(len(self.locations)):
            x0, y0 = self.locations[i]
            x1 = x0 * np.cos(yaw_change) + y0 * np.sin(yaw_change) - displacement
            y1 = -x0 * np.sin(yaw_change) + y0 * np.cos(yaw_change)
            self.locations[i] = np.array([x1, y1])
        self.locations.appendleft(np.array([0,0]))

    def reset(self):
        self.locations.clear()

# 读取侦测框数据
def read_box_data(path):
    data = pd.read_csv(path ,header=None,sep = ' ')
    data.columns = TRACKING_COLUMN_NAMES
    data.loc[data.type.isin(['Truck','Van','Tram']),'type'] = 'Car'
    data = data[data.type.isin(['Car','Cyclist','Pedestrian'])]
    return data

# 画图片上2d盒子
def draw_2d_box(data,image, frame):
    type_array = np.array(data.loc[data['frame'] == frame, ['type']]) 
    box_array = np.array(data.loc[data['frame'] == frame, ['bbox_left','bbox_top','bbox_right','bbox_bottom']])
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

# 读取imu数据以画出路线
def read_imu(imu_path):
    df = pd.read_csv(imu_path, header=None, sep=' ')
    df.columns = IMU_COLUMN_NAMES
    return df

# 计算点到线段的最短距离
def distance_point_to_segment(P,A,B):
  """
  calculates the min distance of point P to a segment AB.
  return min distance and point q
  """

  AP = P-A
  BP = P-B
  AB = B-A
  # 锐角，投影点在线段上
  if np.dot(AB,AP)>=0 and np.dot(-AB,BP)>=0:
    return np.abs(np.cross(AP,AB))/np.linalg.norm(AB), np.dot(AP,AB)/np.dot(AB,AB)*AB+A
  # 否则线段外
  d_PA = np.linalg.norm(AP)
  d_PB = np.linalg.norm(BP)
  if d_PA < d_PB:
    return d_PA, A 
  return d_PB, B

# 计算两个长方体最短距离
def min_distance_cuboids(cub1,cub2):
  """
  compute min dist between two non-overlapping cuboids of shape (8,4)
  """

  minD = 1e5
  for i in range(4):
    for j in range(4):
    #   d, Q = distance_point_to_segment(cub1[i,:2], cub2[j,:2], cub2[j+1,:2])
      d, Q = distance_point_to_segment(cub1[i,:2], cub2[:2,j], cub2[:2, j+1])
      if d < minD:
        minD = d
        minP = cub1[i,:2]
        minQ = Q
  for i in range(4):
    for j in range(4):
      d, Q = distance_point_to_segment(cub1[i,:2], cub2[:2,j], cub2[:2, j+1])
    #   d, Q = distance_point_to_segment(cub1[i,:2], cub2[j,:2], cub2[j+1,:2])
      if d < minD:
        minD = d
        minP = cub2[i,:2]
        minQ = Q
  return minP, minQ, minD