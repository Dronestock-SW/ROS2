from glob import glob
from setuptools import find_packages, setup

setup(
    name='drone_demo', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/drone_demo']),
        ('share/drone_demo', ['package.xml']),
        ('share/drone_demo/config', glob('config/*.json')),
        ('share/drone_demo/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
    maintainer='Dronestock', maintainer_email='user@todo.todo',
    description='Reusable demo inputs without a flight interface.',
    license='Proprietary',
    entry_points={'console_scripts': [
        'demo_export = drone_demo.export:main',
        'demo_node = drone_demo.node:main',
        'demo_mission_node = drone_demo.mission_node:main',
        'demo_cycle = drone_demo.cycle:main',
    ]},
)
