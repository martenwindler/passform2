from setuptools import setup
import os
from glob import glob

package_name = 'passform_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jasper Wilhelm',
    maintainer_email='wil@biba.uni-bremen.de',
    description='ROS2 package managing a modular assembly station.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_manager = passform_base.base_manager:main',
        ],
    },
)
