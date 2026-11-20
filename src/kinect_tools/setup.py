from setuptools import setup

package_name = 'kinect_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sriharsha',
    maintainer_email='you@example.com',
    description='Kinect v2 audio publisher, point cloud node, viewers and diagnostics',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_publisher = kinect_tools.audio_publisher:main',
            'pointcloud_node = kinect_tools.pointcloud_node:main',
            'rgb_viewer = kinect_tools.rgb_viewer:main',
            'depth_viewer = kinect_tools.depth_viewer:main',
            'ir_viewer = kinect_tools.ir_viewer:main',
            'audio_listener = kinect_tools.audio_listener:main',
            'audio_saver = kinect_tools.audio_saver:main',
            'pointcloud_viewer = kinect_tools.pointcloud_viewer:main',
            'unified_viewer = kinect_tools.unified_viewer:main',
            'integration_test = kinect_tools.integration_test:main',
            'diagnose_topics = kinect_tools.diagnose_topics:main',
        ],
    },
)
