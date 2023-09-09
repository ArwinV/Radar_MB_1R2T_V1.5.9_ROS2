from setuptools import find_packages, setup

package_name = 'lidar_testnode'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arwin',
    maintainer_email='arwin_verhoeven@hotmail.com',
    description='Basic ROS2 node that publishes LaserScan messages',
    license='GNU GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "lidar = lidar_testnode.lidar_mb_1r2t_publisher:main"
        ],
    },
)
