from setuptools import setup

package_name = 'bt_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name + '/launch', ['launch/bumpgo.launch.py']),
        ('share/' + package_name + '/resource', ['resource/bt_examples']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rodrigo Pérez-Rodríguez',
    maintainer_email='rodrigo.perez@urjc.es',
    description='Comportamiento de ejemplos de py_trees en ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sequence = bt_examples.sequence:main',
            'reactive_sequence = bt_examples.reactive_sequence:main',
            'fallback = bt_examples.fallback:main',
            'reactive_fallback = bt_examples.reactive_fallback:main',
            'decorator = bt_examples.decorator:main',
        ],
    },
)
