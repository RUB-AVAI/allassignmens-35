from setuptools import setup
import os
from glob import glob

package_name = 'camera_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", "camera_pkg"), glob("launch/launchfile.py"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='denis.meral@rub.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "camera_node = camera_pkg.camera_node:main",
            "image_processing_node = camera_pkg.image_processing_node:main",
            "gui_node = camera_pkg.gui_node:main",
            "occupancy_map_node = camera_pkg.occupancy_map_node:main",
            "turtlebot_slam_node = camera_pkg.turtlebotslam_node:main"
        ]
    },
)
