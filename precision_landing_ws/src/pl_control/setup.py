from setuptools import find_packages, setup

package_name = 'pl_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berke',
    maintainer_email='tnberkec@gmail.com',
    description='Landing controller and MAVSDK bridge for GPS-denied precision landing',
    license='MIT',
    entry_points={
        'console_scripts': [
            'landing_controller_node = pl_control.landing_controller_node:main',
        ],
    },
)
