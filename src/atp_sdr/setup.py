from glob import glob
from setuptools import setup

package_name = 'atp_sdr'

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
            'sdr_receiver = atp_sdr.nodo_sdr:main',
            'sdr_gate = atp_sdr.nodo_gate:main',
            'sdr_beacon = atp_sdr.nodo_beacon:main',
            'sdr_offline = atp_sdr.cadena:main',
        ],
    },
)
