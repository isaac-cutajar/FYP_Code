from setuptools import find_packages, setup

package_name = 'led_control'

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
		'led_serial_node = led_control.led_serial_node:main',
		'led_serial_node2 = led_control.led_serial_node2:main',
		'ssmcolour = led_control.ssmcolour:main',
		'polar = led_control.polar:main',
		'polar2 = led_control.polar2:main',
		'polar3 = led_control.polar3:main',
		'polar4 = led_control.polar4:main',
		'polar5 = led_control.polar5:main',
		'polar6 = led_control.polar6:main',
		'polar7 = led_control.polar7:main',
		'polar8 = led_control.polar8:main',
		'polar9 = led_control.polar9:main',
		'polar10 = led_control.polar10:main',
         ],
    },
)
