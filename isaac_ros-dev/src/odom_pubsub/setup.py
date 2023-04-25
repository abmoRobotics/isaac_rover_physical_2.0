from setuptools import setup

package_name = 'odom_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bogdan-Stefan Miron',
    maintainer_email='bmiron21@student.aau.dk',
    description='Package that can take the output of the /odom and/or /odom_visual and use it',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = odom_pubsub.publisher_member_function:main',
            'listener = odom_pubsub.subscriber_member_function:main',
            'odometry = odom_pubsub.check_odom:main',
            'imu = odom_pubsub.check_imu:main',
            'camera_noise_subscriber = odom_pubsub.camera_noise_subscriber:main',
            'camera_noise_publisher = odom_pubsub.camera_noise_publisher:main'
        ],
    },
)
