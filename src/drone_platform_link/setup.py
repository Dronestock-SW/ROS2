from setuptools import find_packages, setup

setup(
    name='drone_platform_link', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/drone_platform_link']),
        ('share/drone_platform_link', ['package.xml']),
        ('share/drone_platform_link/deploy', [
            'deploy/dronestock-companion.service', 'deploy/companion.env.example',
        ]),
    ],
    install_requires=['setuptools', 'websockets==13.1'],
    tests_require=['pytest'],
    maintainer='Dronestock', maintainer_email='user@todo.todo',
    description='Platform mission reception and read-only ROS telemetry.',
    license='Proprietary',
    entry_points={'console_scripts': ['platform_link = drone_platform_link.runtime:main']},
)
