from setuptools import setup
from glob import glob

package_name = 'gp7_robot_description'

setup(
    name=package_name,
    version='0.0.0',

    # ✅ VERY IMPORTANT: no Python packages here
    packages=[],

    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        ('share/' + package_name + '/meshes/collision', glob('meshes/collision/*')),
        ('share/' + package_name + '/meshes/visual', glob('meshes/visual/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros-industrial',
    maintainer_email='TODO:',
    description='URDF description package for GP7 Robot manipulator with gripper',
    license='BSD',
    entry_points={
        'console_scripts': [
        ],
    },
)
