from setuptools import setup, find_packages

package_name = 'dynamixel_sdk'

setup(
    name=package_name,
    version='3.8.1',
    packages=find_packages(where='src'),  # 'src' 내부에서 패키지 찾기
    package_dir={'': 'src'},  # 패키지의 루트 디렉토리를 'src'로 지정
    data_files=[
        ('share/ament_index/resource_index/packages', ['package.xml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Dynamixel SDK for ROS 2 (C++ and Python)',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
