from pathlib import Path
from setuptools import find_packages, setup

PACKAGE = 'fr3_bolt_cell'
data = [
    ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
    ('share/' + PACKAGE, ['package.xml', 'README.md', 'THIRD_PARTY.md', 'LICENSE']),
]
for directory in ('config', 'launch', 'urdf', 'meshes', 'rviz', 'docs'):
    for path in sorted(Path(directory).rglob('*')):
        if path.is_file() and '__pycache__' not in path.parts:
            data.append(('share/' + PACKAGE + '/' + str(path.parent), [str(path)]))

setup(
    name=PACKAGE, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=data, install_requires=['setuptools', 'PyYAML', 'numpy'],
    zip_safe=False, maintainer='FR3 cell maintainer',
    maintainer_email='maintainer@example.com',
    description='FR3 bolt inspection simulation cell', license='MIT',
    entry_points={'console_scripts': [
        'generate_world = fr3_bolt_cell.world:main',
        'gripper = fr3_bolt_cell.gripper:main',
        'publish_scene = fr3_bolt_cell.planning_scene:main',
        'check_sim = fr3_bolt_cell.check_sim:main',
    ]},
)
