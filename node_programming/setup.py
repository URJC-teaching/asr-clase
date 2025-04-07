from setuptools import setup

package_name = 'node_programming'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'rclpy_lifecycle', 'launch_ros'],
    package_data={
        package_name: ['resource/*', 'launch/*'],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/pubsub.launch.py", "launch/lc_pubsub.launch.py"]),
    ],
    entry_points={
        'console_scripts': [
            'publisher_node = node_programming.publisher_node:main',
            'subscriber_node = node_programming.subscriber_node:main',
            'lifecycle_publisher_node = node_programming.lifecycle_publisher_node:main',
        ],
    },
)
    