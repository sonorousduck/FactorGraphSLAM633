from setuptools import setup

package_name = 'pointcloud_to_kml'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A package to save point clouds as KML files for overlaying on Google Maps.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'save_pointcloud_to_kml = pointcloud_to_kml.save_pointcloud_to_kml:main',
        ],
    },
)
