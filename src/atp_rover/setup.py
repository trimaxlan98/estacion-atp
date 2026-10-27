from glob import glob
from setuptools import setup

package_name = 'atp_rover'

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
            'rover_dinamica = atp_rover.nodo_dinamica:main',
            'rover_offline = atp_rover.cli:main',
            'rover_navegador = atp_rover.nodo_navegador:main',
            'rover_enlace = atp_rover.nodo_enlace:main',
        ],
    },
)
