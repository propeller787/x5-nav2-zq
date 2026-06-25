from setuptools import setup
import os
from glob import glob

package_name = 'wheelchair_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Howard',
    maintainer_email='dev@example.com',
    description='Wheelchair navigation package (sandbox-friendly fake robot).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_robot = wheelchair_nav.scripts.fake_robot:main',
        ],
    },
)
