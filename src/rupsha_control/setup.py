from glob import glob

from setuptools import find_packages, setup

package_name = 'rupsha_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/ardupilot', glob('config/ardupilot/*.parm')),
    ],
    install_requires=['setuptools', 'pymavlink', 'pyserial'],
    zip_safe=True,
    maintainer='Muhammad Fahim Faisal',
    maintainer_email='fh1m.dev@gmail.com',
    description='Rupsha control: board owner, /rupsha/move server, verb facade',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manager = rupsha_control.rover_manager_node:main',
            'bringup_check = rupsha_control.bringup_check:main',
            'param_audit = rupsha_control.param_audit:main',
        ],
    },
)
