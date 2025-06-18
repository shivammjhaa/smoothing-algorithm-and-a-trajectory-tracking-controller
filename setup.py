from setuptools import find_packages, setup

package_name = 'trajectory_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shivam Kumar',
    maintainer_email='shivam@example.com',
    description='Trajectory follower for differential drive robot using ROS 2 and Gazebo.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_follower = trajectory_follower.follower_node:main',
        ],
    },
)
