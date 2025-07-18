from setuptools import find_packages, setup

package_name = 'turtle_parking_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cctv_parking_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = turtle_parking_bot.camera_publisher:main',
            'cctv_detector = turtle_parking_bot.cctv_detector:main',
            'cctv_find_empty = turtle_parking_bot.cctv_find_empty:main',
            'roi_test = turtle_parking_bot.select_roi_from_topic:main',
            'nav_to_empty_spot = turtle_parking_bot.nav_to_empty_spot:main',
            'center_controller = turtle_parking_bot.center_controller:main'
        ],
    },
)
