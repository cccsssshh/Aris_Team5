from setuptools import find_packages, setup

package_name = 'kiosk_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/ui', ['ui/kiosk.ui']),
        ('share/' + package_name + '/models', ['models/gesture_model_v2.keras', 'models/gesture_model_v2.hdf5', 'models/age_model.hdf5'])
    ],
    install_requires=[
        'setuptools',
        'deepface==0.0.92',
        'opencv-python==4.8.0.76',
        'tensorflow==2.15.0',
    ],
    zip_safe=True,
    maintainer='k',
    maintainer_email='kgt6467@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kiosk = kiosk_package.kiosk:main',
            'deeplearning_model = kiosk_package.deeplearning_model:main'

        ],
    },
)
