from setuptools import setup

package_name = 'perception_supervisor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ronan Le Guenne',
    maintainer_email='ronan.le-guenne@polytech-lille.net',
    description='State machine de fusion AprilTag + YOLO + LiDAR pour le tracking du Spot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'perception_supervisor = perception_supervisor.perception_supervisor_node:main',
        ],
    },
)
