from setuptools import find_packages, setup

package_name = 'fd_xm_infer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', ['launch/xm_infer.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='donghyeon',
    maintainer_email='jdhyeon280812@gmail.com',
    description='X-Mobility 파인튜닝 체크포인트(.ckpt) 추론 노드 — 카메라 · odom · 경로 → /cmd_vel',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'xm_infer = fd_xm_infer.node:main'
        ],
    },
)
