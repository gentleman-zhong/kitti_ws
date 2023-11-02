import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
import pandas as pd
from sensor_msgs.msg import Imu,NavSatFix
import tf_transformations
import threading

DATA_PATH = '/home/zhangzhong/kitti_ws_use/2011_09_26_drive_0005_sync/2011_09_26/2011_09_26_drive_0005_sync'
COUNT = 154
IMU_COLUMN_NAMES = ['lat', 'lon', 'alt', 'roll', 'pitch', 'yaw', 'vn', 've', 'vf', 'vl', 'vu', 'ax', 'ay', 'az', 'af','al', 'au', 'wx', 'wy', 'wz', 'wf', 'wl', 'wu', 'posacc', 'velacc', 'navstat', 'numsats', 'posmode','velmode', 'orimode']


class ImuGpsPublisherNode(Node):

    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        self.imu_pub = self.create_publisher(Imu, "kitti_imu", 10)   # 创建发布者对象（消息类型、话题名、队列长度）
        self.pub_rate = self.create_rate(5)
        self.gps_pub = self.create_publisher(NavSatFix, "kitti_gps", 10)
        self.thread = threading.Thread(target=self.__publish_imu_gps)
        self.thread.start()


    def __publish_imu_gps(self):
        imu_gps_data_list = [DATA_PATH + '/oxts/data/%010d.txt'%i for i in range(COUNT)]        
        
        imu = Imu()
        imu.header.frame_id = 'map'
        imu.header.stamp = self.get_clock().now().to_msg()

        gps = NavSatFix()
        gps.header.frame_id = 'map'
        gps.header.stamp = self.get_clock().now().to_msg()
        frame = 0
        while rclpy.ok():
            # 读取单个文件imu数据
            imu_gps_data = pd.read_csv(imu_gps_data_list[frame], header=None, sep=' ').astype(float)
            imu_gps_data.columns = IMU_COLUMN_NAMES

            gps.latitude = float(imu_gps_data.lat.iloc[0]) 
            gps.longitude = float(imu_gps_data.lon.iloc[0]) 
            gps.altitude = float(imu_gps_data.alt.iloc[0]) 
            
            

    # Imu数据属性设置
            q = tf_transformations.quaternion_from_euler(float(imu_gps_data.roll.iloc[0]), float(imu_gps_data.pitch.iloc[0]), float(imu_gps_data.yaw.iloc[0])) # prevent the data from being overwritten
            imu.orientation.x = q[0]
            imu.orientation.y = q[1]
            imu.orientation.z = q[2]
            imu.orientation.w = q[3]
            imu.linear_acceleration.x = float(imu_gps_data.af.iloc[0])
            imu.linear_acceleration.y = float(imu_gps_data.al.iloc[0])
            imu.linear_acceleration.z = float(imu_gps_data.au.iloc[0])
            imu.angular_velocity.x = float(imu_gps_data.wf.iloc[0])
            imu.angular_velocity.y = float(imu_gps_data.wl.iloc[0])
            imu.angular_velocity.z = float(imu_gps_data.wu.iloc[0])
            
            self.imu_pub.publish(imu)
            self.gps_pub.publish(gps)


            self.get_logger().info(f'{imu_gps_data_list[frame]} is published')
            frame += 1
            frame %= COUNT
            self.pub_rate.sleep() 
            


def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = ImuGpsPublisherNode("imu_gps_publisher_node")     # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口