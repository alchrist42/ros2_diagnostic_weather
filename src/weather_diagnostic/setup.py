from setuptools import find_packages, setup
from glob import glob

package_name = 'weather_diagnostic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, glob('launch/*launch.py')),
        ('share/' + package_name, glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arh',
    maintainer_email='arh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'weather_diagnostic_native = weather_diagnostic_native.weather_diagnostic:main',
            'weather_diagnostic = weather_diagnostic.weather_diagnostic:main'
        ],
    },
)
