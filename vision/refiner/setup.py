from setuptools import setup

package_name = 'apriltag_refiner'

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
    description='Raffinement de pose AprilTag avec la profondeur D435i',
    license='MIT',
    entry_points={
        'console_scripts': [
            'refiner = apriltag_refiner.refiner_node:main',
            'pose_fuser = apriltag_refiner.pose_fuser_node:main',
        ],
    },
)
