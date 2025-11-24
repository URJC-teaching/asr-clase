from setuptools import setup
import os
from glob import glob

package_name = 'bt_bumpgo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),        
        (os.path.join('share', package_name, 'bt_xml'), glob('bt_xml/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rodrigo Pérez-Rodríguez',
    maintainer_email='rodrigo.perez@urjc.es',
    description='Comportamiento bump-go con py_trees en ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bumpgo = bt_bumpgo.bumpgo:main',
            'bumpgo_side = bt_bumpgo.bumpgo_side:main',
            'bumpgo_groot = bt_bumpgo.bumpgo_groot:main',
        ],
    },
)