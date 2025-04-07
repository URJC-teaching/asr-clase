from setuptools import setup

package_name = 'tf_seeker'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tf_seeker.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rodrigo Pérez-Rodríguez',
    maintainer_email='rodrigo.perez@urjc.es',
    description='TF example for ROS 2',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_seeker_node = tf_seeker.tf_seeker_node:main',
            'tf_publisher_node = tf_seeker.tf_publisher_node:main',
        ],
    },
)
