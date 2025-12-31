from setuptools import find_packages, setup

package_name = 'wt901c_imu'

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
    maintainer='omi',
    maintainer_email='sdhudu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wit_imu_node_mag = wt901c_imu.imu_node:main',
	        'wit_basic_imu_node = wt901c_imu.basic_imu_node:main',
            'test_imu_node = wt901c_imu.full_imu_node:main',
            'wit_imu_node = wt901c_imu.wit_full:main',
            'wit_imu_with_cal_node = wt901c_imu.imu_with_cal:main',
        ],
    },
)
