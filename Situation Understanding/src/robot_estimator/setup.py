from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot_estimator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lukas',
    maintainer_email='lgerstla@stud.hs-heilbronn.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robotControllerNode = robot_estimator.robotControllerNode:main',
            'ekfNode = robot_estimator.ekfNode:main',
            'VisualizationNode = robot_estimator.visualizationNode:main',
            'cameraDetectionNode = robot_estimator.cameraDetectionNode:main',
            'lidarDetectionNode = robot_estimator.lidarDetectionNode:main',
        ],
    },
)
