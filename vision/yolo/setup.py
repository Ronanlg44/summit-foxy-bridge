from setuptools import setup

package_name = 'yolo_detector'

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
    description='Detection YOLO du Spot via modele ONNX, publie Detection2DArray',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_detector = yolo_detector.yolo_detector_node:main',
        ],
    },
)
