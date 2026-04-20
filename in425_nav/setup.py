from setuptools import setup
import os
from glob import glob

package_name = 'in425_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share/" + package_name), glob("launch/*_launch.py"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johvany Gustave',
    maintainer_email='johvany.gustave@ipsa.fr',
    description='This package contains the navigation nodes, namely RRT implementation and the path following algorithm',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rrt_node.py = in425_nav.rrt_node:main",
            "motion_node.py = in425_nav.motion_node:main"
        ],
    },
)
