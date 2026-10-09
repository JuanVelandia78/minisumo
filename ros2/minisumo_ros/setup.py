from setuptools import find_packages, setup

package_name = 'minisumo_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan Velandia',
    maintainer_email='est.juand.velandia@unimilitar.edu.co',
    description='Nodos ROS2 del robot minisumo teleoperado',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'robot_bridge = minisumo_ros.robot_bridge:main',
            'diff_kinematics = minisumo_ros.diff_kinematics:main',
            'data_logger = minisumo_ros.data_logger:main',
            'glove_teleop = minisumo_ros.glove_teleop:main',
            'routines = minisumo_ros.routines:main',
        ],
    },
)
