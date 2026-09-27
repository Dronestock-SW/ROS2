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
        'uwb_node = drone_uwb.integration.node:main',
        'uwb_px4_bridge = drone_uwb.integration.bridge:main',
        'uwb_replay = drone_uwb.integration.replay:main',
        'uwb_bench_probe = drone_uwb.integration.bench_probe:main',
        'uwb_pipeline = drone_uwb.processing.runner:main',
        'uwb_static_a = drone_uwb.processing.experiments.static_a:main',
        'uwb_h80_b = drone_uwb.processing.experiments.h80_b:main',
        'uwb_baseline_a = drone_uwb.processing.experiments.baseline_a:main',
    ]},
)
