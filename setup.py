import os
from glob import glob
from setuptools import setup

package_name = 'odrivelib'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name,'config'), glob(os.path.join('config', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chris',
    maintainer_email='weixinychris@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_odrive_srv = odrivelib.node_odrive_srv:main',
            'node_odrive_cli = odrivelib.node_odrive_cli:main',   
            'node_odrive_sub = odrivelib.node_odrive_sub:main',     
            'node_odrive_pub = odrivelib.node_odrive_pub:main',
            ],
    },
)