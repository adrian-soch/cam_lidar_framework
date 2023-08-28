import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'fusion_engine'

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
        (os.path.join('share', package_name), glob('config/*/*.*'))
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
            'camera_processor = fusion_engine.camera_processing_node:main'
        ],
    },
)
