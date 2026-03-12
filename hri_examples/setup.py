from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hri_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Recurso para indexado de paquetes
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Archivo de definición del paquete
        ('share/' + package_name, ['package.xml']),
        # Archivos de launch instalables
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Archivos de configuración
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.txt'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roi',
    maintainer_email='rodrigo.perez@urjc.es',
    description='ROS 2 examples for Human-Robot Interaction. Includes TTS action client.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'say = hri_examples.say_client_node:main',
            'repeat = hri_examples.repeat_node:main',
            'generate_response_node = hri_examples.generate_response_node:main',
            'nao_hri_example = hri_examples.nao_hri_example:main',
            'hri_example = hri_examples.hri_example:main',
            'hri_example2 = hri_examples.hri_example2:main',
            'hri_example3 = hri_examples.hri_example3:main',
            'hri_example_client = hri_examples.hri_example_client:main',
            'hri_example2_client = hri_examples.hri_example2_client:main',
            'hri_example3_client = hri_examples.hri_example3_client:main',
        ],
    },
)
