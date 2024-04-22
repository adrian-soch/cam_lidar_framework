from setuptools import setup

package_name = 'lidar_obj_classifier'

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
    maintainer='adrian',
    maintainer_email='adrian-soch@github.com',
    description='Perfrom object classifcation for on point cloud data.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_classifier = lidar_obj_classifier.lidar_obj_classifier:main',
            'classifier_validation = lidar_obj_classifier.classifier_validation:main',
        ],
    },
)
