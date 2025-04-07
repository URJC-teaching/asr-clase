from setuptools import setup

package_name = 'fsm_bumpgo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bumpgo.launch.py']),
        ('share/' + package_name + '/resource', ['resource/' + package_name])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rodrigo Pérez-Rodríguez',
    maintainer_email='rodrigo.perez@urjc.es',
    description='Bumpgo behavior implemented with an FSM',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bump_go_node = fsm_bumpgo.bumpgo_node:main',
        ],
    },
)
