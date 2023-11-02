import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
import threading
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from .my_kitti_util import *              #这种导入方式和ros2的特性有关系，可以在setup.py里面设置，具体参考https://blog.csdn.net/scarecrow_sun/article/details/127589380?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522168811155116800182166106%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=168811155116800182166106&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-2-127589380-null-null.142^v88^control,239^v2^insert_chatgpt&utm_term=ros2%E4%B8%AD%E8%87%AA%E5%B7%B1%E5%86%99%E7%9A%84%E4%BB%A3%E7%A0%81%E6%80%8E%E4%B9%88%E4%BD%BF%E7%94%A8&spm=1018.2226.3001.4187

DATA_PATH = '/home/zhangzhong/kitti_ws_use/2011_09_26_drive_0005_sync/2011_09_26/2011_09_26_drive_0005_sync'
TRACKING_PATN = '/home/zhangzhong/kitti_ws_use/training/label_02/0000.txt'
COUNT = 154

class ImagePublisherNode(Node):

    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        self.pub = self.create_publisher(Image, "kitti_image", 10)   # 创建发布者对象（消息类型、话题名、队列长度）
        # 图像发布函数可以以独立的频率发布图像，而不会受到ROS节点主循环的限制,要确保在多线程编程中正确处理线程同步和共享数据
        self.thread = threading.Thread(target=self.__publish_image)
        self.thread.start()

    def __publish_image(self):
        image_list = [DATA_PATH + '/image_02/data/%010d.png'%i for i in range(COUNT)]
        pub_rate = self.create_rate(10)
        # frame用来表示是第几帧数
        frame = 0
        boxes_data = read_box_data(path=TRACKING_PATN)
        cv_bridge = CvBridge()
        while rclpy.ok():
            image = cv2.imread(image_list[frame])
            draw_2d_box(data=boxes_data,image=image,frame=frame)
            self.pub.publish(cv_bridge.cv2_to_imgmsg(image,"bgr8"))
            self.get_logger().info(f'{image_list[frame]} is published')
            frame += 1
            frame %= COUNT
            pub_rate.sleep()



def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = ImagePublisherNode("image_publisher_node")     # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口