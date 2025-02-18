from setuptools import find_packages, setup

package_name = 'dynamixel_sdk_examples_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    author='Wonho Yun',
    author_email='ywh@robotis.com',
    description='Dynamixel SDK examples in Python',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'read_write_node = dynamixel_sdk_examples_py.read_write_node:main',
        ],
    },
)
