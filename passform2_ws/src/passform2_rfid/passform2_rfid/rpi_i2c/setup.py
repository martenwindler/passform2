import os
from glob import glob
from setuptools import setup
from setuptools import find_packages


package_name = 'rpi_i2c'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),#[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
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
            'rpi_i2c_di = rpi_i2c.rpi_i2c_di:main',
            'rpi_i2c_do = rpi_i2c.rpi_i2c_do:main',
            'rpi_i2c_ai = rpi_i2c.rpi_i2c_ai:main',
            'rpi_i2c_ao = rpi_i2c.rpi_i2c_ao:main',
            #'rpi_gpio_us = rpi_i2c.rpi_gpio_us:main',
            'gpio_us_node = rpi_i2c.gpio_us_node:main',
            'angle_sensor = rpi_i2c.angle_sensor:main',
            'compartment = rpi_i2c.compartment:main'
        ],
    },
)
