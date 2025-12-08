from setuptools import setup

package_name = 'textbook_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    py_modules=['nodes.physics_manager', 'nodes.sensor_simulators'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Textbook Maintainer',
    maintainer_email='maintainer@textbook.org',
    description='Textbook Gazebo simulation package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'physics_manager = nodes.physics_manager:main',
            'sensor_simulator = nodes.sensor_simulators:main',
        ],
    },
)