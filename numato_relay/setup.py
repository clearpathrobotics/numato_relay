import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'numato_relay'

setup(
    name=package_name,
    version='0.1.3',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='administrator',
    maintainer_email='rfaultless@clearpathrobotics.com',
    description='Driver for controlling Numato Labs USB relay PCBAs',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = numato_relay.numato_relay:main',
        ],
    },
)
