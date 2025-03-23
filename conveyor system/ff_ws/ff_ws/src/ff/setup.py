from setuptools import find_packages, setup

package_name = 'ff'

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
    maintainer='keynmouse',
    maintainer_email='keynmouse@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui=ff.gui_node:main',
            'conveyor=ff.conveyor_node:main',
            'global_camera=ff.global_camera_node:main',
            'robot_camera=ff.robot_camera_node:main',
            'robot=ff.robot_node:main',
            'login_test=ff.login_test:main',
        ],
    },
)
