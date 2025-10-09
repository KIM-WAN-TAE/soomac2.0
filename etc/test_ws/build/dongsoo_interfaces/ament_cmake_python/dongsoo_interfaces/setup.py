from setuptools import find_packages
from setuptools import setup

setup(
    name='dongsoo_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('dongsoo_interfaces', 'dongsoo_interfaces.*')),
)
