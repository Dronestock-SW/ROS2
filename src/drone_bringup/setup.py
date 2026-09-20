from glob import glob

from setuptools import find_packages, setup

package_name = 'drone_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'qr_decoder_node = drone_bringup.qr_decoder_node:main',
            'qr_reader_node = drone_bringup.qr_reader_node:main',
            'qr_fallback_node = drone_bringup.qr_fallback_node:main',
            'qr_parser_node = drone_bringup.qr_parser_node:main',
        ],
    },
)
