from setuptools import setup

package_name = 'ros2_intention_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wintermelon',
    maintainer_email='yiwen.ma@epfl.ch',
    description='Detect intention of user',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'detector = ros2_intention_detector.intention_detector_function:main',
             'listener = ros2_intention_detector.intention_subscriber_function:main',
        ],
    },
)
