from setuptools import find_packages, setup

package_name = 'tuos_examples'

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
    maintainer='tom',
    maintainer_email='t.howard@sheffild.ac.uk',
    description='Examples for the COM2009 ROS2 Course',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_pose = tuos_examples.robot_pose:main'
        ],
    },
)
