from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wall_follower'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joao-sousa',
    maintainer_email='joao-sousa@todo.todo',
    description='Left wall-following robot using LiDAR',
    license='MIT',
    entry_points={
        'console_scripts': [
            'wall_follower_node = wall_follower.wall_follower_node:main',
        ],
    },
)
