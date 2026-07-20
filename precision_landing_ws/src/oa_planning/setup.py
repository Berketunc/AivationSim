import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'oa_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         glob('launch/*.py')),
        ('share/' + package_name + '/config',
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berke',
    maintainer_email='tnberkec@gmail.com',
    description='3D A* path planning over the octomap',
    license='MIT',
    entry_points={
        'console_scripts': [
            'planner_node = oa_planning.planner_node:main',
        ],
    },
)
