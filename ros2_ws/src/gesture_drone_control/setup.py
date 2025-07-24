from setuptools import find_packages, setup

package_name = 'gesture_drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/gesture_drone_control', ['package.xml']),
    ('share/ament_index/resource_index/packages', ['resource/gesture_drone_control']),
    ('share/gesture_drone_control/models', ['models/gesture_classifier.pkl', 'models/labels.json']),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Parikshit',
    maintainer_email='parik.sonwane06@gmail.com',
    description='Gesture-based drone control with ROS 2 and MAVSDK',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_publisher = gesture_drone_control.gesture_publisher:main',
            'drone_command_node = gesture_drone_control.drone_command_node:main',
        ],
    },
)