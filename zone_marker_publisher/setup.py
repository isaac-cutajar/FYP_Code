from setuptools import find_packages, setup

package_name = 'zone_marker_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='isaac2',
    maintainer_email='isaac2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'GUI = zone_marker_publisher.GUI:main',
		'panel = zone_marker_publisher.panel:main',
		'markers = zone_marker_publisher.markers:main',
        ],
    },
)
