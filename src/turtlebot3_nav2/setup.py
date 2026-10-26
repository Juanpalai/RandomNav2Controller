from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'turtlebot3_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan Pablo Lagos Idrobo',
    maintainer_email='juan.pablo.lagosi@gmail.com',
    description='Drive a TurtleBot3 to random goals using Nav2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_controller = turtlebot3_nav2.nav2_controller:main'
        ],
    },
)
