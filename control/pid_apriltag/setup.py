from setuptools import setup

package_name = 'pid_apriltag'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='Ronan Le Guenne',
    maintainer_email='ronan.le-guenne@polytech-lille.net',
    description='PID asservissement Summit XL -> Spot via /spot_pose_in_summit + WebSocket',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pid_apriltag = pid_apriltag.pid_apriltag_node:main',
            'pid_apriltag_with_ws = pid_apriltag.wrapper:main',
        ],
    },
)
