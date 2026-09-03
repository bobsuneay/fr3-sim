from setuptools import setup

package_name = 'fairino_dualarm_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim.launch.py']),
        ('share/' + package_name + '/launch', ['launch/gazebo.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/dual_arm.urdf.xacro']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/' + package_name + '/rviz', ['rviz/dual_arm.rviz']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'sim_controller = fairino_dualarm_sim.sim_controller:main',
            'sim_vision = fairino_dualarm_sim.sim_vision:main',
            'grasp_demo = fairino_dualarm_sim.grasp_demo:main',
        ],
    },
)
