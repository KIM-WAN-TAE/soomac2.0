from setuptools import find_packages, setup

package_name = 'vision_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
        'pyrealsense2',
        'ultralytics',
        'tf-transformations'
    ],
    zip_safe=True,
    maintainer='temp_id',
    maintainer_email='kwt9882@naver.com',
    description='ROS2 vision node for tool detection using RealSense camera and YOLO',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = vision_node.tool_detect:main',
        ],
    },
)
