from setuptools import setup

package_name = 'fusion_3d'

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
    description='Fuses 3D image detections and 3D LiDAR detections into 1 track list.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion_3d_node = fusion_3d.fusion_3d_node:main',
            'fusion_3d_viz_node = fusion_3d.fusion_3d_visualizer_node:main',
        ],
    },
)
