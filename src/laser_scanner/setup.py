from setuptools import find_packages, setup

package_name = 'robot_control'

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
    maintainer='LoanTTM',
    maintainer_email='tr.loan232@gmail.com',
    description='Robot with IR sensor and 4 wheel motor control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'irsensor = robot_control.ir_sensor_node:main',
            'motors = robot_control.motor_controller:main',
        ],
    },
)
