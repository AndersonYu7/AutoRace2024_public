from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autorace'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'), glob(os.path.join('launch','*launch.[pxy][yma]'))),
        (os.path.join('share',package_name,'config'), glob(os.path.join('config/*.yaml'))),
        (os.path.join('share',package_name,'config'), glob(os.path.join('config/*.rviz')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iclab',
    maintainer_email='iclab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_lane_v2 = autorace.detect_lane_v2:main',
            'control_lane = autorace.control_lane:main',
            'hsv_param_adjustment = autorace.hsv_param_adjustment:main',
        ],
    },
)
