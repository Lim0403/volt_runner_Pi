from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'mecanum_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament index & package.xml (필수)
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),

        # 🔻 launch 파일들을 설치 경로에 포함
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # 🔻 config/*.yaml도 함께 배포 (주석 해제!)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jm',
    maintainer_email='jm@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rpm_receiver_node = mecanum_pkg.rpm_receiver_node:main',
            'input_node = mecanum_pkg.mecanum_input_node:main',
            'ik_node = mecanum_pkg.mecanum_ik_node:main',
            'mecanum_serial_bridge = mecanum_pkg.mecanum_serial_bridge:main',
            'wheel_rpm_sender = mecanum_pkg.wheel_rpm_sender:main',
            'odom_publisher = mecanum_pkg.odom_publisher:main',
        ],
    },
)

