from glob import glob
import os

from setuptools import find_packages, setup

# I added these for installing launch, src, and config files to share folder locations
package_name = 'pid_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py'))),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/src', glob('src/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='trucker',
    maintainer_email='tmorris@umn.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'steering_pd_controller = pid_python.steering_pd_controller:main',
        ],
    },
)
