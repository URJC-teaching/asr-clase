from setuptools import find_packages, setup

package_name = 'tf_square_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roi',
    maintainer_email='rodrigo.perez@urjc.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'tf_square = tf_square_motion.tf_square_node:main',
        'tf_square2 = tf_square_motion.tf_square_node2:main',
    ],
},

)
