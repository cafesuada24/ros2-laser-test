import os
from setuptools import setup
from glob import glob

package_name = 'hardware_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LoanTTM',
    maintainer_email='tr.loan232@gmail.com',
    description='Robot with IR sensor and 4 wheel motor control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'irsensor = hardware_controller.irsensor:main',
            'motors_controller = hardware_controller.motors_controller:main',
            'ultrasonic = hardware_controller.ultrasonic:main'
        ],
    },
)
