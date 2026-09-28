from setuptools import find_packages, setup

package_name = 'fd_sim_warmup'

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
    maintainer='donghyeon',
    maintainer_email='jdhyeon280812@gmail.com',
    description='워밍업 — Isaac Sim 샘플 씬(carter_warehouse_navigation, Nova Carter)을 3D LiDAR 로 장애물 회피 주행. 첫 폐루프 예행연습',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_wander = fd_sim_warmup.lidar_wander:main'
        ],
    },
)
