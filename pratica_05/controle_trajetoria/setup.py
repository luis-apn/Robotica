import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'controle_trajetoria'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='leomax.filho@gmail.com',
    description='Práticas 03/04: controle de pose, missão e trajetória em 8 (com sensores)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_monitor = controle_trajetoria.odom_monitor:main',
            'pose_controller = controle_trajetoria.pose_controller:main',
            'mission = controle_trajetoria.mission:main',
            'trajectory_8 = controle_trajetoria.trajectory_8:main',
        ],
    },
)
