from setuptools import setup

package_name = 'rl_camera'

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
    maintainer='xavier',
    maintainer_email='jknuds19@student.aau.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'RLModel_node = exomy.RLModel_node:main',
            'Camera_node = exomy.Camera_node:main'
        ],
    },
)
