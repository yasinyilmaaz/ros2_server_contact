from setuptools import find_packages, setup

package_name = 'data_contact_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch dosyaları:
        ('share/' + package_name + '/launch', ['launch/telemetry_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yasin',
    maintainer_email='yasn.866644@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #"data_contact_node = data_contact_py_pkg.data_contact_node:main",
            #"battery_status_node = data_contact_py_pkg.battery_status_node:main",
            'map_to_jpeg_node = data_contact_py_pkg.MapToJpegNode:main',
            'speed_estimator_node = data_contact_py_pkg.speed_estimator_node:main',
            'obstacle_detector_node = data_contact_py_pkg.obstacle_detector_node:main',
            'battery_passthrough_node = data_contact_py_pkg.battery_passthrough_node:main',
            'server_bridge_node = data_contact_py_pkg.server_bridge_node:main',
        ],
    },
)
