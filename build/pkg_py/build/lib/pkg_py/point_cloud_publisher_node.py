# https://blog.csdn.net/scarecrow_sun/article/details/129159971?ops_request_misc=&request_id=&biz_id=102&utm_term=ros2%E6%98%AF%E6%80%8E%E4%B9%88%E8%AF%BB%E5%8F%96pointcloud2&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-1-129159971.142^v88^control,239^v2^insert_chatgpt&spm=1018.2226.3001.4187

#开头双下划线私有属性私有方法
# 待解决:路线不恢复为0的问题;
import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from sensor_msgs.msg import PointCloud2
import numpy as np
from sensor_msgs_py import point_cloud2 as pcl2
from std_msgs.msg import Header
import threading
from visualization_msgs.msg import Marker,MarkerArray        #标记库，画视线
from .my_kitti_util import * 
from rclpy.duration import Duration
from geometry_msgs.msg import Point

DATA_PATH = '/home/zhangzhong/kitti_ws_use/2011_09_26_drive_0005_sync/2011_09_26/2011_09_26_drive_0005_sync'
TRACKING_PATN = '/home/zhangzhong/kitti_ws_use/training/label_02/0000.txt'
CALIBRATION_FILE_PATH = '/home/zhangzhong/kitti_ws_use/2011_09_26_calib/2011_09_26/'
COUNT = 154
# connect vertic
LINES = [[0, 1], [1, 2], [2, 3], [3, 0]] # lower face
LINES+= [[4, 5], [5, 6], [6, 7], [7, 4]] # upper face
LINES+= [[4, 0], [5, 1], [6, 2], [7, 3]] # connect lower face and upper face
LINES+= [[4, 1], [5, 0]] # front face and draw x

DETECTION_COLOR_DICT = {'Car':(255,255,0), 'Cyclist':(255,0,255), 'Pedestrian':(0,255,255)}
EGO_CAR = np.array([[2.15, 0.9, -1.73], [2.15, -0.9, -1.73], [-1.95, -0.9, -1.73],[-1.95,0.9,-1.73], [2.15,0.9,-0.23], [2.15,-0.9,-0.23],[-1.95,-0.9,-0.23],[-1.95,0.9,-0.23]])


class PointCloudPublisherNode(Node):

    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        self.point_cloud_pub = self.create_publisher(PointCloud2, "kitti_point_cloud", 10)   # 创建发布者对象（消息类型、话题名、队列长度）
        self.box_3d_pub = self.create_publisher(MarkerArray, "kitti_point_cloud_3d_boxes", 10)   # 创建发布者对象（消息类型、话题名、队列长度）
        self.route_pub = self.create_publisher(MarkerArray,"route_of_object", 10)
        self.distance_pub = self.create_publisher(MarkerArray,"distance_between_object", 10)
        self.thread = threading.Thread(target=self.__publish_point_cloud)
        self.thread.start()

    
    def __publish_3d_boxes(self,boxes_data, frame, box_3d_pub):
            # 发布侦测框和track_id
            this_frame_boxes_data = np.array(boxes_data.loc[boxes_data['frame'] == frame, ['height','width','length','pos_x','pos_y','pos_z','rot_y']]) #对应帧坐标数据
            type_array = np.array(boxes_data.loc[boxes_data['frame'] == frame, ['type']]) #对应帧标注类型数据
            track_ids_array = np.array(boxes_data.loc[boxes_data['frame'] == frame, ['track_id']]) #对应帧track_id数据
            this_frame_velo_boxes_data = [] #这是转换后的激光雷达坐标系下数据
            if this_frame_boxes_data.size > 0:
                for box_data in this_frame_boxes_data:
                    this_frame_velo_boxes_data += [compute_rect_to_velo(CALIBRATION_FILE_PATH,compute_3d_box_cam2(*box_data))]           
            marker_array = MarkerArray()
            for i, corners_3d_velo in enumerate(this_frame_velo_boxes_data):
                # 画出侦测框
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = i
                marker.action = Marker.ADD
                marker.lifetime = Duration(seconds=0.2).to_msg()
                marker.type = Marker.LINE_LIST
                b,g,r = DETECTION_COLOR_DICT[type_array[i][0]]
                marker.color.r = r/255.0
                marker.color.g = g/255.0
                marker.color.b = b/255.0
                marker.color.a = 1.0
                marker.scale.x = 0.2 #scale of line    
                marker.points = []
                for l in LINES:
                    marker.points.append(Point())
                    marker.points[-1].x = corners_3d_velo[0][l[0]]
                    marker.points[-1].y = corners_3d_velo[1][l[0]]
                    marker.points[-1].z = corners_3d_velo[2][l[0]]

                    marker.points.append(Point())
                    marker.points[-1].x = corners_3d_velo[0][l[1]]
                    marker.points[-1].y = corners_3d_velo[1][l[1]]
                    marker.points[-1].z = corners_3d_velo[2][l[1]]
                marker_array.markers.append(marker)

                # 画出track_id
                track_id = track_ids_array[i]
                text_marker = Marker()
                text_marker.header.frame_id = 'map'
                text_marker.header.stamp = self.get_clock().now().to_msg()

                text_marker.id = int(track_id + 1000)
                text_marker.action = Marker.ADD
                text_marker.lifetime = Duration(seconds=0.2).to_msg()
                text_marker.type = Marker.TEXT_VIEW_FACING

                text_marker.pose.position.x = corners_3d_velo[0][4]
                text_marker.pose.position.y = corners_3d_velo[1][4]
                text_marker.pose.position.z = corners_3d_velo[2][4]+0.5
                text_marker.text = str(track_id[0])

                text_marker.scale.x = 1.0
                text_marker.scale.y = 1.0
                text_marker.scale.z = 1.0
                
                text_marker.color.r = r/255.0
                text_marker.color.g = g/255.0
                text_marker.color.b = b/255.0
                text_marker.color.a = 1.0
                marker_array.markers.append(text_marker)
            box_3d_pub.publish(marker_array)

    def __publish_car_route(self, route_pub, locations):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.action = Marker.ADD
        marker.lifetime = Duration(seconds=0.2).to_msg()
        marker.type = Marker.LINE_LIST

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.scale.x = 0.2

        marker.points = []
        for i in range(len(locations)-1):
            marker.points.append(Point())
            marker.points[-1].x = float(locations[i][0])
            marker.points[-1].y = float(locations[i][1])
            marker.points[-1].z = 0.0

            marker.points.append(Point())
            marker.points[-1].x = float(locations[i+1][0])
            marker.points[-1].y = float(locations[i+1][1])
            marker.points[-1].z = 0.0            

        marker_array.markers.append(marker)
        route_pub.publish(marker_array)

    def __publish_distance(self, distance_pub, minPQDs):
        marker_array = MarkerArray()
        for i, (minP,minQ,minD) in enumerate(minPQDs):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.action = Marker.ADD
            marker.lifetime = Duration(seconds=0.2).to_msg()
            marker.type = Marker.LINE_LIST
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.5
            marker.scale.x = 0.1

            marker.points = []
            marker.points.append(Point())
            marker.points[-1].x = float(minP[0])
            marker.points[-1].y = float(minP[1])
            marker.points[-1].z = 0.0
            marker.points.append(Point())
            marker.points[-1].x = float(minQ[0])
            marker.points[-1].y = float(minQ[1])
            marker.points[-1].z = 0.0
            marker_array.markers.append(marker)

            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.id = i+1000
            text_marker.action = Marker.ADD
            text_marker.lifetime = Duration(seconds=0.2).to_msg()
            text_marker.type = Marker.TEXT_VIEW_FACING
            p = (minP+minQ)/2.0
            text_marker.pose.position.x = p[0]
            text_marker.pose.position.y = p[1]
            text_marker.pose.position.z = 0.0
            text_marker.text = '%.2f'%minD
            text_marker.scale.x = 1.0
            text_marker.scale.y = 1.0
            text_marker.scale.z = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 0.8
            marker_array.markers.append(text_marker)
        distance_pub.publish(marker_array)


    def __publish_point_cloud(self):
        point_cloud_list = [DATA_PATH + '/velodyne_points/data/%010d.bin'%i for i in range(COUNT)]
        header = Header()   #常用的消息类型
        header.frame_id = 'map'   # 坐标系
        header.stamp = self.get_clock().now().to_msg()  #时间戳
        
        boxes_data = read_box_data(path=TRACKING_PATN)
        
        imu_path_list = [DATA_PATH + '/oxts/data/%010d.txt'%i for i in range(COUNT)]
        ego_car = Object()
        pre_imu_data = None

        pub_rate = self.create_rate(10)
        frame = 0        
        while rclpy.ok():
            # 发布点云
            point_cloud = np.fromfile(point_cloud_list[frame],dtype=np.float32).reshape(-1,4)
            pcl2_msg = pcl2.create_cloud_xyz32(header,point_cloud[:,:3])
            self.point_cloud_pub.publish(pcl2_msg)           
            
            # 发布3d侦测框
            self.__publish_3d_boxes(boxes_data=boxes_data,frame=frame,box_3d_pub=self.box_3d_pub)

            # 发布路线
            imu_data = read_imu(imu_path=imu_path_list[frame])
            if pre_imu_data is not None:
                move_distance = 0.1*np.linalg.norm(imu_data[['vf', 'vl']])
                yaw_change = float(imu_data.yaw.iloc[0]-pre_imu_data.yaw.iloc[0])
                ego_car.update(displacement=move_distance, yaw_change=yaw_change)
            pre_imu_data = imu_data
            self.__publish_car_route(route_pub=self.route_pub, locations=ego_car.locations)
            
            # 发布车与各个物体的距离
            this_frame_boxes_data = np.array(boxes_data.loc[boxes_data['frame'] == frame, ['height','width','length','pos_x','pos_y','pos_z','rot_y']]) #对应帧坐标数据
            MinPQDs = []
            for box_3d in this_frame_boxes_data:
                MinPQDs +=[min_distance_cuboids(cub1=EGO_CAR,cub2=compute_rect_to_velo(CALIBRATION_FILE_PATH,compute_3d_box_cam2(*box_3d)))] 
            self.__publish_distance(distance_pub=self.distance_pub, minPQDs=MinPQDs)


            self.get_logger().info(f'{point_cloud_list[frame]} is published')
            frame += 1
            if frame == 154:
                ego_car.reset
                pre_imu_data = None
                frame = 0
            pub_rate.sleep() 


 




def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = PointCloudPublisherNode("point_cloud_publisher_node")     # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
