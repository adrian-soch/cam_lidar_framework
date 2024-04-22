from setuptools import setup

package_name = 'fusion_module'

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
            'fusion_node = fusion_module.fusion_2d_node:main',
            'fusion_viz_node = fusion_module.fusion_visualizer_node:main',
            'detection2csv_node = fusion_module.detection2csv_node:main'
        ],
    },
)
