import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'oa_vio'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         glob('launch/*.py')),
        ('share/' + package_name + '/config/aviationsim',
         glob('config/aviationsim/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berke',
    maintainer_email='tnberkec@gmail.com',
    description='OpenVINS calibration config and world-frame alignment',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vio_odom_to_world = oa_vio.vio_odom_to_world:main',
            'vio_divergence_watchdog = oa_vio.vio_divergence_watchdog:main',
        ],
    },
)
