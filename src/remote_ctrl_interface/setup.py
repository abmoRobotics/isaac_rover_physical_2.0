import os
from glob import glob
from setuptools import setup

package_name = 'remote_ctrl_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='abmo19@student.aau.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'live_feed = remote_ctrl_interface.live_feed:main',
            'zed_camera_node = remote_ctrl_interface.zed_camera_node:main',
        ],
    },
)
