from glob import glob
from setuptools import find_packages, setup

setup(
    name='drone_mission', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/drone_mission']),
        ('share/drone_mission', ['package.xml']),
        ('share/drone_mission/config', glob('config/*')),
        ('share/drone_mission/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    maintainer='Dronestock', maintainer_email='user@todo.todo',
    description='PX4 flight mission sequencing and local web trial tools.',
    license='Proprietary',
    entry_points={'console_scripts': [
        'flight_mission = drone_mission.node:main',
        'local_flight_web = drone_mission.local_web:main',
    ]},
)
