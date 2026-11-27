from glob import glob
from setuptools import setup

setup(
    name='pgo_gridmap_projector',
    version='0.1.0',
    packages=['pgo_gridmap_projector'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/pgo_gridmap_projector']),
        ('share/pgo_gridmap_projector/launch',
            glob('launch/*.launch.py')),
        ('share/pgo_gridmap_projector/config',
            glob('config/*.yaml')),
        # Offline verification harness. Not a node -- run it by hand while the
        # launch file is up; see its docstring for usage.
        ('lib/pgo_gridmap_projector/scripts',
            glob('scripts/*.py')),
        ('share/pgo_gridmap_projector',
            ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Claude',
    maintainer_email='claude@anthropic.com',
    description='Projects the PGO-corrected full map into a GridMap elevation layer.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'projector_node = pgo_gridmap_projector.projector_node:main',
        ],
    },
)
