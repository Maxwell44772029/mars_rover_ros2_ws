from setuptools import setup, find_packages

package_name = 'mars_rover_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    include_package_data=True,
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup_sim.launch.py']),
        ('share/' + package_name + '/config', ['config/octomap.rviz']),
        ('share/' + package_name + '/scripts', ['scripts/pose_to_tf.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maxwellg',
    maintainer_email='you@example.com',
    description='Hybrid Mars Rover Simulation Launch',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'project_octomap_to_costmap = mars_rover_sim.project_octomap_to_costmap:main', 'simple_astar_navigator = mars_rover_sim.simple_astar_navigator:main',
        ],
    },
)