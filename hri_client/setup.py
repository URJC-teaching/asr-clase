from setuptools import find_packages, setup

package_name = 'hri_client'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ASR Course',
    maintainer_email='rodrigo.perez@urjc.es',
    description='Cliente reutilizable para servicios de simple_hri (STT, TTS, Extract, YesNo)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
