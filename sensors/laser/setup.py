from setuptools import setup
import os
from glob import glob

package_name = 'laser'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rodrigo Pérez Rodríguez',
    maintainer_email='tucorreo@ejemplo.com',
    description='Paquete Python ROS 2 para detección de obstáculos con láser',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detector_node = laser.obstacle_detector_node:main',
            'obstacle_detector_node_no_tf = laser.obstacle_detector_node_no_tf:main',
        ],
    },
)
