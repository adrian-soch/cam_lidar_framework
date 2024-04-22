from setuptools import setup

package_name = 'fusion_2d'

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
    description='Fuses 2D image detections and projected 2D LiDAR detections into 1 track list.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion_node = fusion_2d.fusion_2d_node:main',
            'fusion_viz_node = fusion_2d.fusion_visualizer_node:main',
            'detection2csv_node = fusion_2d.detection2csv_node:main'
        ],
    },
)
