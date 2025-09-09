from setuptools import find_packages
from setuptools import setup

setup(
    name='zeus_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('zeus_interfaces', 'zeus_interfaces.*')),
)
