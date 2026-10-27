from glob import glob
from setuptools import setup

package_name = 'sat_adcs'

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
            'adcs_sim = sat_adcs.nodo_planta:main',
            'adcs_controller = sat_adcs.nodo_controlador:main',
            'adcs_monitor = sat_adcs.nodo_monitor:main',
            'adcs_recorder = sat_adcs.nodo_grabador:main',
            'adcs_offline = sat_adcs.escenario:main',
        ],
    },
)
