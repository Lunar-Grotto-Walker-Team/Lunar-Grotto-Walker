from setuptools import find_packages, setup

package_name = 'pointlio_cloud_relay'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@example.com',
    description=(
        'Relay point_lio clouds from the body frame to base_footprint '
        'for Nav2 costmaps.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cloud_to_base = pointlio_cloud_relay.cloud_to_base:main',
            'ground_filter = pointlio_cloud_relay.ground_filter:main',
            'exploration_map_relay = '
            'pointlio_cloud_relay.exploration_map_relay:main',
        ],
    },
)
