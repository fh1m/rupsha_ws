from glob import glob

from setuptools import find_packages, setup

package_name = 'rupsha_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhammad Fahim Faisal',
    maintainer_email='fh1m.dev@gmail.com',
    description='Rupsha planner: autonomy executive, mission DSL, CLI',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rupsha = rupsha_planner.cli:main',
            'mission = rupsha_planner.mission:main',
            'executive = rupsha_planner.executive_node:main',
        ],
    },
)
