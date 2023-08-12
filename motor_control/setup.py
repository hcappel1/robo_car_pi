from setuptools import setup
import os
from glob import glob

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hcappel1',
    maintainer_email='hcappel1@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_count_node = motor_control.encoder_count:main',
            'wheel_omega_node = motor_control.wheel_omega:main',
            'pid_omega_node = motor_control.pid_omega:main',
            'encoder_data_node = motor_control.encoder_data:main',
            'motor_test_node = motor_control.motor_test:main',
        ],
    },
)
