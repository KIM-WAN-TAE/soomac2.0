from setuptools import find_packages, setup

package_name = 'dongsoo_control_pkg'

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
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_connect=dongsoo_control_pkg.dongsoo_motor_connect:main',
            'data_hub=dongsoo_control_pkg.dongsoo_data_hub:main',
            'monitoring_node=dongsoo_control_pkg.dongsoo_monitoring_node:main',
            'server_node=dongsoo_control_pkg.dongsoo_server_node:main',
            'client_node=dongsoo_control_pkg.dongsoo_client_node:main',
            'gripper_node=dongsoo_control_pkg.dongsoo_gripper_node:main'
        ],
    },
)
