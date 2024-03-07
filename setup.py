from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'get_target_point'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join('share', package_name, 'utils'), glob('utils/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='7zjatl7@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_recognition = get_target_point.action_recogintion:main',
            'target_point = get_target_point.target_point:main',
            'async_target_point = get_target_point.async_target_point:main'
        ],
    },
)
