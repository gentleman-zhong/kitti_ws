# https://blog.csdn.net/scarecrow_sun/article/details/129159971?ops_request_misc=&request_id=&biz_id=102&utm_term=ros2%E6%98%AF%E6%80%8E%E4%B9%88%E8%AF%BB%E5%8F%96pointcloud2&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-1-129159971.142^v88^control,239^v2^insert_chatgpt&spm=1018.2226.3001.4187

import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from sensor_msgs.msg import PointCloud2,Image
import numpy as np
from sensor_msgs_py import point_cloud2 as pcl2
import time
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2


DATA_PATH = '/home/zhangzhong/kitti_ws_use/2011_09_26_drive_0005_sync/2011_09_26/2011_09_26_drive_0005_sync'


class PointCloudAndImagePublisherNode(Node):

    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        self.point_cloud_pub = self.create_publisher(PointCloud2, "kitti_point_cloud", 10)   # 创建发布者对象（消息类型、话题名、队列长度）
        self.image_pub = self.create_publisher(Image, "kitti_image", 10)   # 创建发布者对象（消息类型、话题名、队列长度）

        self.publish_point_cloud_and_image(self.point_cloud_pub, self.image_pub)

    def publish_point_cloud_and_image(self, point_cloud_pub, image_pub):
        point_cloud_list = [DATA_PATH + '/velodyne_points/data/%010d.bin'%i for i in range(154)]
        header = Header()   #常用的消息类型
        header.frame_id = 'map'   # 坐标系
        header.stamp = self.get_clock().now().to_msg()  #时间戳

        cv_bridge = CvBridge()
        image_list = [DATA_PATH + '/image_02/data/%010d.png'%i for i in range(154)]

        flag = 0
        while rclpy.ok():
            point_cloud = np.fromfile(point_cloud_list[flag],dtype=np.float32).reshape(-1,4)
            pcl2_msg = pcl2.create_cloud_xyz32(header,point_cloud[:,:3])
            point_cloud_pub.publish(pcl2_msg)

            image = cv2.imread(image_list[flag])
            image_pub.publish(cv_bridge.cv2_to_imgmsg(image,"bgr8"))

            self.get_logger().info(f'{point_cloud_list[flag]} is published')
            self.get_logger().info(f'{image_list[flag]} is published')
            flag += 1
            flag %= 154
            time.sleep(0.1)






def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = PointCloudAndImagePublisherNode("point_cloud_and_image_publisher_node")     # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口

