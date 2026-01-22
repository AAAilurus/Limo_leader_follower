from setuptools import find_packages, setup

package_name = 'lf_turtlesim'

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
    maintainer='labpc',
    maintainer_email='labpc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'leader_safety_filter = lf_turtlesim.leader_safety_filter:main',
        'leader_path_recorder = lf_turtlesim.leader_path_recorder:main',
        'follower_path_tracker = lf_turtlesim.follower_path_tracker:main',
        'follower_safety_filter = lf_turtlesim.follower_safety_filter:main',
    ],
},

)
