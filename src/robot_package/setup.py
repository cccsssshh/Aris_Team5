from setuptools import find_packages, setup

package_name = 'robot_package'

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
    maintainer='lee',
    maintainer_email='dltmdgh1346@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'KioskToRobot = robot_package.KioskToRobot:main',
            'RobotToKiosk = robot_package.RobotToKiosk:main',
            'RobotToAdminiPub = robot_package.RobotToAdminiPub:main',
            'RobotToAdminiSub = robot_package.RobotToAdminiSub:main',
        ],
    },
)
