from setuptools import setup
from glob import glob
package_name = 'mid360_passage_detector'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Local User',
    maintainer_email='maintainer@example.com',
    description='Standalone MID360 relative passage detector',
    license='Apache-2.0',
    entry_points={'console_scripts': ['passage_detector = mid360_passage_detector.node:main']},
)
