from setuptools import find_packages, setup

package_name = 'vff_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vff_2d.launch.py']),
        ('share/' + package_name + '/launch', ['launch/vff_3d.launch.py']),
        ('share/' + package_name + '/launch', ['launch/yolo_class_3d.launch.py']),
        ('share/' + package_name + '/launch', ['launch/yolo_class_2d.launch.py']),
        ('share/' + package_name + '/launch', ['launch/yolo_class_3d_alt.launch.py']),
        ('share/' + package_name + '/launch', ['launch/obstacle_detector.launch.py']),
        ('share/' + package_name + '/launch', ['launch/full_vff_2d.launch.py']),
        ('share/' + package_name + '/launch', ['launch/full_vff_3d.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roi',
    maintainer_email='rodrigo.perez@urjc.es',
    description='VFF controller with attractive and repulsive vectors',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vff_controller_node = vff_control.vff_controller_node:main',
            'obstacle_detector_node = vff_control.obstacle_detector_node:main',
            'obstacle_detector_node_no_tf = vff_control.obstacle_detector_node_no_tf:main',
            'yolo_class_detector_node_2d = vff_control.yolo_class_detector_node_2d:main',
            'yolo_class_detector_node_3d = vff_control.yolo_class_detector_node_3d:main',
            'yolo_class_detector_node_3d_alt = vff_control.yolo_class_detector_node_3d_alt:main',
        ],
    },
)
