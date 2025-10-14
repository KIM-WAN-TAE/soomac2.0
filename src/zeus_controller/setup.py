from setuptools import find_packages, setup

package_name = 'zeus_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pc',
    maintainer_email='jyw010704@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'connect_zeus = zeus_controller.zeus_connect:main',
            'server_zeus = zeus_controller.zeus_server:main',
            'client_zeus = zeus_controller.zeus_client:main',
            'gripper_zeus = zeus_controller.zeus_block_gripper:main',
            'main_control = zeus_controller.zeus_main_control:main',
        ],
    },
)
