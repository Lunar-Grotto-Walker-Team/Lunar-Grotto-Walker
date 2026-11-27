from setuptools import setup

package_name = 'elevation_fast_postprocessor'

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
    maintainer='Claude',
    maintainer_email='claude@anthropic.com',
    description=(
        'O(N) numpy/OpenCV post-processing for the PGO projector elevation '
        'grid, replacing the per-cell grid_map_filters chain.'),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'postprocessor_node = '
            'elevation_fast_postprocessor.postprocessor_node:main',
        ],
    },
)
