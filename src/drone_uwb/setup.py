from glob import glob
from setuptools import find_packages, setup

setup(
    name='drone_uwb', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/drone_uwb']),
        ('share/drone_uwb', ['package.xml']),
        ('share/drone_uwb/config', glob('config/*')),
        ('share/drone_uwb/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
    maintainer='Dronestock', maintainer_email='user@todo.todo',
    description='UWB horizontal observations for PX4 EKF2.',
    license='Proprietary',
    entry_points={'console_scripts': [
        'uwb_node = drone_uwb.node:main',
        'uwb_px4_bridge = drone_uwb.bridge:main',
        'uwb_replay = drone_uwb.replay:main',
        'uwb_bench_probe = drone_uwb.bench_probe:main',
        'uwb_pipeline = drone_uwb.preimu.runner:main',
    ]},
)
