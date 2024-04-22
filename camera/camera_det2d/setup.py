import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'camera_det2d'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # Automatically add all packages
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name),
         glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('config/*.*')),
        (os.path.join('share', package_name), glob('config/*/*.*')),
        (os.path.join('share', package_name), glob('weights/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adrian',
    maintainer_email='adrian-soch@github.com',
    description='Perfroms image-based 2D object detection.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_processor = camera_det2d.camera_processing_node:main'
        ],
    },
)
