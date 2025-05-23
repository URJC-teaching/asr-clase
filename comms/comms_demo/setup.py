from setuptools import setup

setup(
    name='comms_demo',
    version='0.0.1',
    packages=['comms_demo'],  # <- DEBE coincidir con carpeta
    data_files=[
        ('share/comms_demo', ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='Rodrigo Pérez Rodríguez',
    maintainer_email='rodrigo.perez@urjc.es',
    description='Demo con cliente y servidor de la acción GenerateInformation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'action_client = comms_demo.action_client:main',
            'action_server = comms_demo.action_server:main',
            'service_client = comms_demo.service_client:main',
            'service_server = comms_demo.service_server:main'
        ],
    },
)
