from setuptools import find_packages, setup

package_name = 'dongsoo_py_pkg'

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
            'trajectory_test = dongsoo_py_pkg.trajectory_test:main',
            'one_way_trajectory = dongsoo_py_pkg.one_way_trajectory:main',
            'motor_dummy = dongsoo_py_pkg.motor_dummy:main',
            'data_hub_py = dongsoo_py_pkg.data_hub:main',
            'monitoring_hub = dongsoo_py_pkg.monitoring_hub:main',
        ],
    },
)
