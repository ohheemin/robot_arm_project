from setuptools import find_packages, setup

package_name = 'dynamixel_control'

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
    maintainer='wt',
    maintainer_email='wt@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'example = dynamixel_control.example:main',  
            'motor_control = dynamixel_control.dynamixel_ctl:main',
            'move = dynamixel_control.move:main',
            'move_point_to_point = dynamixel_control.move_point_to_point:main',
            'move_point_ikpy = dynamixel_control.move_point_ikpy:main',
            'move_by_orientation = dynamixel_control.move_by_orientation:main',
            'draw_circle = dynamixel_control.draw_circle:main'
        ],
    },
)
