from glob import glob
from setuptools import setup

package_name = 'atp_mcu'

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
            'mcu_merge = atp_mcu.mcu_merge:main',
            'mcu_lockstep = atp_mcu.mcu_lockstep:main',
        ],
    },
)
