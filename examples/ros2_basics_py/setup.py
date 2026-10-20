from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ros2_basics_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Ximenes',
    maintainer_email='danielximenes164@gmail.com',
    description='ROS 2 basics in Python: topics with QoS, service, action, lifecycle',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talker = ros2_basics_py.talker:main',
            'listener = ros2_basics_py.listener:main',
            'add_two_ints_server = ros2_basics_py.add_two_ints_server:main',
            'add_two_ints_client = ros2_basics_py.add_two_ints_client:main',
            'fibonacci_server = ros2_basics_py.fibonacci_server:main',
            'fibonacci_client = ros2_basics_py.fibonacci_client:main',
            'managed_talker = ros2_basics_py.managed_talker:main',
        ],
    },
)
