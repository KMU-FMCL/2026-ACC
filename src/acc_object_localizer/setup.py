from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'acc_object_localizer'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jsm',
    maintainer_email='jsm@example.com',
    description='Object localizer for QCar2 using YOLO detections and RealSense depth',
    license='MIT',
    tests_require=['pytest'],
)
