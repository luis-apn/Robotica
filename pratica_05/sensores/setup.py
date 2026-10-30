import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'sensores'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luis',
    maintainer_email='luisnetoalemanha@gmail.com',
    description='Prática 05: GPS → XYZ e pose bruta do robô pelos sensores (GPS + IMU)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gps_to_xyz = sensores.gps_to_xyz:main',
            'pose_sensores = sensores.pose_sensores:main',
        ],
    },
)
