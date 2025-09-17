from setuptools import setup

package_name = 'occupancy_grid_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='surendrakoganti',
    maintainer_email='kumarsurendra.1000@gmail.com',
    description='Package description',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hsc_gridmap = occupancy_grid_map.gridmap_hsc:test_gridmap_node_checkered',
            'hsc_gridmap_figure = occupancy_grid_map.gridmap_hsc:test_gridmap_node_figure',
            'hsc_mapping_sample = occupancy_grid_map.mapping_hsc:mapping_with_sample',
            'hsc_mapping_laser = occupancy_grid_map.mapping_hsc:mapping_with_laser_scan',
        ],
    },
)
