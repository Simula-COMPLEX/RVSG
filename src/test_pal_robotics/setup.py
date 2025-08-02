from setuptools import find_packages, setup

package_name = 'test_pal_robotics'

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
    maintainer='complexse25',
    maintainer_email='complexse25@simula.no',
    description='test_pal_robotics',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_pal_robotics = test_pal_robotics.test_pal_robotics:main',
            'spawn_model = test_pal_robotics.spawn_model:main',
            'cmd_behaviors = test_pal_robotics.cmd_behaviors:main',
            'navigate_to_pose_action = test_pal_robotics.navigate_to_pose_action:main',
            'navigate_to_pose_collision_detection = test_pal_robotics.navigate_to_pose_collision_detection:main',
            'waypoint_follower = test_pal_robotics.waypoint_follower:main',
            'waypoint_check = test_pal_robotics.waypoint_check:main',
            'hunav_loader = test_pal_robotics.hunav_loader:main',
            'laser_scan_subscriber = test_pal_robotics.laser_scan_subscriber:main',
            'pointcloud_subscriber = test_pal_robotics.pointcloud_subscriber:main',
            'publish_initial_pose = test_pal_robotics.publish_initial_pose:main',
            'test_hunav = test_pal_robotics.test_hunav:main',
            'waypoints = test_pal_robotics.waypoints:main',
            'image_generate = test_pal_robotics.image_generate:main',
            'navigate_to_pose_requirement = test_pal_robotics.navigate_to_pose_requirement:main',
            'test_best_scenario = test_pal_robotics.test_best_scenario:main',
        ],
    },
)
