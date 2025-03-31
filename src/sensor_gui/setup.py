from setuptools import setup

package_name = 'sensor_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aaditi',
    maintainer_email='aadidkadam415@gmail.com',
    description='Sensor GUI for RHex Robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_gui.py = sensor_gui.sensor_gui:main',
        ],
    },
)

