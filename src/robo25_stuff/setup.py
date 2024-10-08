from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robo25_stuff'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robo25',
    maintainer_email='robo25@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'engine_node = robo25_stuff.engine_node:main',
            'imu_gps_node = robo25_stuff.imu_gps_node:main',
        ],
    },
)
