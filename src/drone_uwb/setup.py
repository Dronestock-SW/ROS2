from glob import glob
from pathlib import Path
from setuptools import find_packages, setup


def config_data_files():
    """Install grouped settings and legacy file aliases into the package share."""
    directories = {}
    for path in sorted(Path('config').rglob('*')):
        if path.is_file():
            directories.setdefault(str(Path('share/drone_uwb') / path.parent), []).append(str(path))
    return list(directories.items())


setup(
    name='drone_uwb', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/drone_uwb']),
        ('share/drone_uwb', ['package.xml']),
        ('share/drone_uwb/launch', glob('launch/*.launch.py')),
    ] + config_data_files(),
    install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
    maintainer='Dronestock', maintainer_email='user@todo.todo',
    description='UWB horizontal observations for PX4 EKF2.',
    license='Proprietary',
    entry_points={'console_scripts': [
        'uwb_node = drone_uwb.integration.ros.node:main',
        'uwb_btf_node = drone_uwb.integration.ros.btf_node:main',
        'uwb_px4_bridge = drone_uwb.integration.ros.bridge:main',
        'uwb_replay = drone_uwb.integration.replay:main',
        'uwb_bench_probe = drone_uwb.integration.ros.bench_probe:main',
        'uwb_manual_capture = drone_uwb.integration.ros.manual_capture:main',
        'manual_observation_streams = drone_uwb.integration.ros.manual_streams:main',
        'uwb_pipeline = drone_uwb.processing.runner:main',
        'uwb_static_a = drone_uwb.processing.experiments.static_a:main',
        'uwb_h80_b = drone_uwb.processing.experiments.h80_b:main',
        'uwb_subset_compare = drone_uwb.processing.experiments.subset_comparison:main',
        'uwb_baseline_a = drone_uwb.processing.experiments.baseline_a:main',
    ]},
)
