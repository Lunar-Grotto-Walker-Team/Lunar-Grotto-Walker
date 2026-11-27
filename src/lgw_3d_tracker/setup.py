from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'lgw_3d_tracker'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='project team',
    maintainer_email='unitree@localhost.localdomain',
    description='Unified 3D tracking across cameras (Phase A-C).',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unified_track_node = lgw_3d_tracker.unified_track_node:main',
        ],
    },
)



