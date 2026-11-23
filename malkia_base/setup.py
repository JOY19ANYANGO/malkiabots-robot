from glob import glob
from setuptools import setup

package_name = 'malkia_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Malkiabots',
    maintainer_email='melkieee26@gmail.com',
    description='Serial bridge between the Arduino Nano and ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'base_node = malkia_base.base_node:main',
        ],
    },
)
