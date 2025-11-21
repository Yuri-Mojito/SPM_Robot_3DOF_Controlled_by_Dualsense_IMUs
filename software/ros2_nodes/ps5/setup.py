from setuptools import find_packages, setup

package_name = 'ps5'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='angelinaecr',
    maintainer_email='angelinaecr@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller_node = ps5.controller_bridge:main',
            'inverse_kinematic = ps5.kinematic_inverse_node:main',
            'steps_node = ps5.steps:main',
            'imu_calibrator = ps5.imu_calibrator:main',
            'imu_logger_node = ps5.imu_logger:main',
        ],
    },
)
