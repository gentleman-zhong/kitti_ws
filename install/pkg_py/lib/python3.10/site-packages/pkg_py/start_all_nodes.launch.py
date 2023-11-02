from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    image_publisher_node = Node(
        package='pkg_py',
        executable='image_publisher_node',
    )

    point_cloud_publisher_node = Node(
        package='pkg_py',
        executable='point_cloud_publisher_node',
    )

    car_veiw_publisher_node = Node(
        package='pkg_py',
        executable='car_veiw_publisher_node',
    )

    return LaunchDescription([image_publisher_node, point_cloud_publisher_node, car_veiw_publisher_node])
