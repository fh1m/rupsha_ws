from glob import glob

from setuptools import find_packages, setup

package_name = 'rupsha_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Muhammad Fahim Faisal',
    maintainer_email='fh1m.dev@gmail.com',
    description='Rupsha localization: shadow RIEKF on SE(2), board health cross-check, opt-in ExtNav',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization = rupsha_localization.localization_node:main',
        ],
    },
)
