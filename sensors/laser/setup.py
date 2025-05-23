from setuptools import setup

package_name = 'laser'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # 👇 NO incluyas 'package_dir' si el nombre del paquete y el directorio son iguales
    data_files=[
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
