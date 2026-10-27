from glob import glob
from setuptools import setup

package_name = 'sat_isl'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'isl_sim = sat_isl.nodo_planta:main',
            'isl_controller = sat_isl.nodo_controlador:main',
            'isl_monitor = sat_isl.nodo_monitor:main',
            'isl_offline = sat_isl.escenario:main',
        ],
    },
)
