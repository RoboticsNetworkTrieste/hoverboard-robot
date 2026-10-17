from glob import glob

from setuptools import setup

package_name = 'hoverboard_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author="Lorenzo D'Auria",
    author_email='lorenzodauria97@gmail.com',
    maintainer="Lorenzo D'Auria",
    maintainer_email='lorenzodauria97@gmail.com',
    description='UDP bridge between the hoverboard ESP32-S3 firmware and ROS2',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = hoverboard_bridge.bridge_node:main',
            'link_probe = hoverboard_bridge.link_probe:main',
        ],
    },
)
