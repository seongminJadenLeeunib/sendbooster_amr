from setuptools import setup
import os
from glob import glob

package_name = 'sendbooster_amr_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='AMR bringup package including motordriver, imu, and odometry nodes.',
    license='License Declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motordriver = sendbooster_amr_bringup.motordriver:main',
            'imu = sendbooster_amr_bringup.imu:main',
            'odometry = sendbooster_amr_bringup.odometry:main',
        ],
    },
)
