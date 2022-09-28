from setuptools import setup

package_name = 'motor_ctrl'

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
    maintainer='Robin Münk',
    maintainer_email='robin.muenk@tum.de',
    description='Package for controlling the Motor via GPIO',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'startstopctrl = motor_ctrl.startstopcontroller:main',
                'smoothctrl = motor_ctrl.smoothcontroller:main',
                'keyboard = motor_ctrl.keyboard_input:main',
                'hardware = motor_ctrl.hardware_controller:main',
                'converter = motor_ctrl.converter:main',
        ],
    },
)
