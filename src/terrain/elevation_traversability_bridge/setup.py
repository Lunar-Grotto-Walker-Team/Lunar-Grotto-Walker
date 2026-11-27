from glob import glob
from setuptools import setup

setup(
    name='elevation_traversability_bridge',
    version='0.1.0',
    packages=['elevation_traversability_bridge'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/elevation_traversability_bridge']),
        ('share/elevation_traversability_bridge/launch',
            glob('launch/*.launch.py')),
        ('share/elevation_traversability_bridge/config',
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Claude',
    maintainer_email='claude@anthropic.com',
    description='Bridge node: elevation Mapping GridMap → terrain traversability OccupancyGrid for Nav2 costmap.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = elevation_traversability_bridge.bridge_node:main',
        ],
    },
)