from setuptools import setup
import os
from glob import glob

package_name = 'pkg_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhangzhong',
    maintainer_email='zhangzhong@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher_node = pkg_py.image_publisher_node:main',
            'point_cloud_publisher_node = pkg_py.point_cloud_publisher_node:main',
            'car_veiw_publisher_node = pkg_py.car_veiw_publisher_node:main',
            'imu_gps_publisher_node = pkg_py.imu_gps_publisher_node:main',
        ],
    },
)
