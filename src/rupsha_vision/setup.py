from glob import glob

from setuptools import find_packages, setup

package_name = 'rupsha_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    # onnxruntime is not a rosdep key; it comes from the venv (onnxruntime on
    # the Pi, onnxruntime-gpu on the dev box) and is only needed for objects.
    install_requires=['setuptools', 'numpy', 'pyyaml'],
    zip_safe=True,
    maintainer='Muhammad Fahim Faisal',
    maintainer_email='fh1m.dev@gmail.com',
    description='Rupsha vision: ArUco posts, ONNX YOLO objects, /rupsha/detections',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision = rupsha_vision.vision_node:main',
        ],
    },
)
