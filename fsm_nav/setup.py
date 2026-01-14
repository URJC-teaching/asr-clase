from setuptools import setup

package_name = 'fsm_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/waypoints.yaml']),
        ('share/' + package_name + '/launch', ['launch/fsm_nav.launch.py']),
        ('share/' + package_name + '/resource', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Ejemplo de máquina de estados para navegación.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fsm_nav_node = fsm_nav.fsm_nav_node:main'
        ],
    },
)
