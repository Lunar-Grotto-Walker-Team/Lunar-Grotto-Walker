from glob import glob

from setuptools import setup

package_name = 'utlidar_terrain_fill'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Claude',
    maintainer_email='claude@anthropic.com',
    description=(
        'Rolling near-field ground fill from /utlidar/cloud_deskewed for '
        'the PGO projector terrain chain.'),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'utlidar_terrain_fill = '
            'utlidar_terrain_fill.fill_node:main',
        ],
    },
)
