from glob import glob

from setuptools import setup

package_name = 'ros2_depthai_package'

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

config_files = sorted(glob('config/*.yaml'))
if config_files:
    data_files.append(('share/' + package_name + '/config', config_files))

launch_files = sorted(glob('launch/*.launch.py'))
if launch_files:
    data_files.append(('share/' + package_name + '/launch', launch_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='htluu@ucsd.edu',
    description='ROS 2 node that publishes RGB frames from a Luxonis DepthAI/OAK camera.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_publisher = ros2_depthai_package.camera_publisher:main'
        ],
    },
)
