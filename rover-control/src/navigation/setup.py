from setuptools import setup
import os
from glob import glob

package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('navigation/launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('navigation/config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'navigator = navigation.navigator:main',
                'trip_planner = navigation.trip_planner:main',
                'mission_ctrl = navigation.mission_control:main',
                'gps = navigation.gps:main',
                'gps_checker = navigation.utility.gps_checker:main',
                'localization = navigation.localization:main',
                'nav_service = navigation.navigation_service:main',
        ],
    },
)
