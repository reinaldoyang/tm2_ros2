from setuptools import setup

package_name = 'waypoint_collector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        ('share/' + package_name + '/launch', ['launch/waypoint_collector.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Reinaldo Yang',
    maintainer_email='reinaldoyang5@gmail.com',
    description='rqt plugin for manual waypoint collection',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
