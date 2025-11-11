from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aydin',
    maintainer_email='92227214+aydindev0@users.noreply.github.com',
    description='YOLO object detection ROS2 package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_test = yolo.yolo_test:main',
            'ultralytics_yolo = yolo.ultralytics_yolo:main',
            'yolo_inference = yolo.yolo_inference:main',
        ],
    },
)
