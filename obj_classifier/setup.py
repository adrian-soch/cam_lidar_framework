from setuptools import setup
import os
from glob import glob

package_name = 'obj_classifier'

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
    maintainer_email='adr.soch@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_classifier = obj_classifier.obj_classifier:main',
            'object_classifier_sub_test = obj_classifier.obj_classifier_subTest:main',
            'tester = obj_classifier.test_caller:main'
        ],
    },
)
