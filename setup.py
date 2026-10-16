from setuptools import find_packages, setup
from glob import glob

package_name = 'quad_nmpc_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                ('share/' + package_name + '/worlds', glob('worlds/*.xml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ajay',
    maintainer_email='j.ajayadit@gmail.com',
    description='Standalone NMPC trajectory tracking for any quadrotor in MuJoCo',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sim_node = quad_nmpc_ros.sim_node:main',
            'nmpc_node = quad_nmpc_ros.nmpc_node:main',
        ],
    },
)
