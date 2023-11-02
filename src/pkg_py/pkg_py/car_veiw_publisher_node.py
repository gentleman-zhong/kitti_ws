import rclpy                                     # ROS2 Python接口库
from rclpy.duration import Duration
from rclpy.node import Node                      # ROS2 节点类
from visualization_msgs.msg import Marker,MarkerArray        #标记库，画视线
import time
import numpy as np
import tf_transformations
from geometry_msgs.msg import Point

# 使用时需要根据自己的照相机视野修改marker角度

class CarViewPublisherNode(Node):

    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        self.pub = self.create_publisher(MarkerArray, "kitti_car_view", 10)   # 创建发布者对象（消息类型、话题名、队列长度）
        self.publish_car_view(self.pub)

    def publish_car_view(self, pub):      
        marker_array = MarkerArray()
        
        # 发布车的视野
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0  #each marker have only one id.
        marker.action = Marker.ADD  #to tell marker the operation is add a new marker
        marker.lifetime = Duration().to_msg() #marker's life time.how long it appears in the frame.            marker.type = Marker.LINE_STRIP # marker's type.Today we use line_strip.        
        marker.type = Marker.LINE_STRIP
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0            
        marker.color.a = 1.0 #apparent degree,transparency
        
        marker.scale.x = 0.2 #scale of line
        # marker.scale.y = 0.01
        # marker.scale.z = 0.01
        
        marker.points = []
        marker.points.append(Point())  # 右前方四十五度点，视野90度
        marker.points[-1].x = float(10)
        marker.points[-1].y = float(-10)
        marker.points[-1].z = float(0)

        marker.points.append(Point())  # (0,0,0)is the location of velodyne LiDAR
        marker.points[-1].x = float(0)
        marker.points[-1].y = float(0)
        marker.points[-1].z = float(0)

        marker.points.append(Point())   # 根据激光雷达坐标，左前方45度
        marker.points[-1].x = float(10)  
        marker.points[-1].y = float(10)
        marker.points[-1].z = float(0)

        marker_array.markers.append(marker)

        # 发布车的模型
        mesh_marker = Marker()
        mesh_marker.header.frame_id = 'map'
        mesh_marker.header.stamp = self.get_clock().now().to_msg()

        mesh_marker.id = -1
        mesh_marker.lifetime = Duration().to_msg()
        mesh_marker.type = Marker.MESH_RESOURCE  # 类型是3d模型
        mesh_marker.mesh_resource = "file:///home/zhangzhong/ROS2/kitti_ws/src/pkg_py/car_model_dae/Car.dae"  

        mesh_marker.pose.position.x = 0.0
        mesh_marker.pose.position.y = 0.0
        mesh_marker.pose.position.z = -1.73  # 坐标系是激光雷达的坐标系，车子大概1.73米，所以设置为-1.73

        q = tf_transformations.quaternion_from_euler(0,0,np.pi/2) #设置旋转
        mesh_marker.pose.orientation.x = q[0]  # 四元数
        mesh_marker.pose.orientation.y = q[1]
        mesh_marker.pose.orientation.z = q[2]
        mesh_marker.pose.orientation.w = q[3]

        mesh_marker.color.r = 1.0
        mesh_marker.color.g = 1.0
        mesh_marker.color.b = 1.0
        mesh_marker.color.a = 1.0

        mesh_marker.scale.x = 0.9
        mesh_marker.scale.y = 0.9
        mesh_marker.scale.z = 0.9

        marker_array.markers.append(mesh_marker)
        while rclpy.ok():
            pub.publish(marker_array)
            self.get_logger().info('car and line published')
            time.sleep(0.1)





def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = CarViewPublisherNode("car_veiw_publisher_node")     # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口

