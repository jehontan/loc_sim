import os
from setuptools import setup
import os
from glob import glob

package_name = 'loc_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'aruco'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jehon',
    maintainer_email='jehontan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localize = loc_sim.localize:main',
            'gen_aruco = aruco.gen_aruco:main'
        ],
    },
)
