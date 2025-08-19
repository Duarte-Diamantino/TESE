from setuptools import setup
import os
from glob import glob
from setuptools import setup, find_packages


package_name = 'pickup_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=['pickup_description'],
    data_files=[
        # Regista o pacote no ament index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Instala o package.xml
        ('share/' + package_name, ['package.xml']),
        # Ficheiros de launch
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # URDF
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
        # Mapas: PNG original, YAML de definição e o novo PGM grayscale
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.png')
            + glob('maps/*.yaml')
            + glob('maps/*.pgm')),
        # Outros config YAML
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='duarte_diamantino',
    maintainer_email='duarte.antonio.eu@gmail.com',
    description='Pickup robot description package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mov_robot_model    = pickup_description.mov_robot_model:main',
            'marker_publisher = pickup_description.marker_publisher:main',
            'rounded_rectangle_marker_publisher = pickup_description.rounded_rectangle_marker_publisher:main',
            'q_path_publisher = pickup_description.q_path_publisher:main',
        ],
    },
)
