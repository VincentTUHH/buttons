from setuptools import setup
import os
from glob import glob

package_name = 'buttons'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Thies Lennart Alff',
    maintainer_email='thies.lennart.alff@tuhh.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_interface = buttons.led_interface_node:main',
            'battery_watcher = buttons.battery_watcher_node:main',
            'button_handler = buttons.button_handler_node:main',
            'button_interface = buttons.button_interface_node:main',
            'buzzer_interface = buttons.buzzer_interface_node:main'
        ],
    },
)
