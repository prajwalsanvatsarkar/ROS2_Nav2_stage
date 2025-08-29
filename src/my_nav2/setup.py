from setuptools import setup
from glob import glob

package_name ='my_nav2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Stage project with goal_pub + Nav2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'goal_pub = my_nav2.goal_pub:main',
            'init_node = my_nav2.init_node:main',
        ],
    },
)

