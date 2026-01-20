import os
from glob import glob
from setuptools import setup

package_name = 'passform_skills'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'skills'), glob('skills/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jasper',
    maintainer_email='wil@biba.uni-bremen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_primitive = passform_skills.example_primitive:main',
            'example_composite = passform_skills.example_composite:main',
            'task_manager = passform_skills.task_manager:main',
        ],
    },
)
