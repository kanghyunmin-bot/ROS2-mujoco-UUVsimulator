from setuptools import setup

package_name = 'hit25_auv'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={package_name: 'scripts'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hit24',
    maintainer_email='hit24@todo.todo',
    description='KMU-HIT 2025 AUV/ROV package (ROS 2 port).',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_node = hit25_auv.main_node:main',
            'rov_tf_broadcaster = hit25_auv.rov_tf_broadcaster:main',
            'ref_point_marker_node = hit25_auv.ref_point_marker_node:main',
            'setpoint_cli = hit25_auv.setpoint_cli:main',
            'controller = hit25_auv.controller:main',
            'buoy_mission = hit25_auv.buoy_mission:main',
            'gate_mission = hit25_auv.gate_mission:main',
            'line_qr_mission = hit25_auv.line_qr_mission:main',
            'hydrophone_mission = hit25_auv.hydrophone_mission:main',
            'return_mission = hit25_auv.return_mission:main',
            'dronecan2mavros_battery = hit25_auv.dronecan2mavros_battery:main',
            'battery_gui = hit25_auv.battery_gui:main',
            'auv_state_gui = hit25_auv.auv_state_gui:main',
        ],
    },
)
