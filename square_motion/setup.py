from setuptools import setup

package_name = 'square_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roi',
    maintainer_email='rodrigo.perez@urjc.es',
    description='A simple ROS 2 node to move a robot in a square.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'square_move = square_motion.square_move_node:main',
        ],
    },
)
