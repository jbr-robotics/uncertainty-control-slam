from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'quality_estimator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/quality_estimator.launch')),
        ('share/' + package_name + '/config', glob('config/quality_estimator.yaml')),
    ],
    install_requires=[
        'setuptools',
        'lupa',
        ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='todo@todo.todo',
    description='Quality Estimator',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quality_estimator = quality_estimator.quality_estimator:main',
        ],
    },
)
