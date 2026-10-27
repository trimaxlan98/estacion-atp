from glob import glob
from setuptools import setup

package_name = 'atp_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'reference_generator = atp_ros.reference_generator:main',
            'pass_predictor = atp_ros.pass_predictor:main',
            'contact_scheduler = atp_ros.contact_scheduler:main',
            'mount_sim = atp_ros.mount_sim:main',
            'pid_controller = atp_ros.pid_controller:main',
            'error_monitor = atp_ros.error_monitor:main',
            'sim_clock = atp_ros.sim_clock:main',
            'link_budget = atp_ros.link_budget:main',
            'campaign_server = atp_ros.campaign_server:main',
            'rf_beacon = atp_ros.rf_beacon:main',
            'autotrack = atp_ros.autotrack:main',
            'recorder = atp_ros.recorder:main',
            'campaign_client = atp_ros.campaign_client:main',
            'lqr_controller = atp_ros.lqr_controller:main',
            'od_estimator = atp_ros.od_estimator:main',
        ],
    },
)
