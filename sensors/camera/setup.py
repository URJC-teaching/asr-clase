from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolo_detection.launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roi',
    maintainer_email='rodrigo.perez@urjc.es',
    description='Package for camera-based YOLO detection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detection_node = camera.yolo_detection_node:main',
        ],
    },
)
