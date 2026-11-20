import os
from glob import glob

from setuptools import setup

package_name = 'kinect_console'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'web'), glob('kinect_console/web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Sriharsha',
    maintainer_email='you@example.com',
    description='Web console for the Kinect v2 topics: camera switcher and mic array view',
    license='MIT',
    entry_points={'console_scripts': ['console = kinect_console.server:main']},
)
