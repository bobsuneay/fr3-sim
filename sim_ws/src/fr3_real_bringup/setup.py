from pathlib import Path
from setuptools import find_packages, setup

PACKAGE = 'fr3_real_bringup'
data = [
    ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
    ('share/' + PACKAGE, ['package.xml', 'README.md', 'THIRD_PARTY.md', 'LICENSE']),
]
for directory in ('config', 'launch', 'urdf', 'meshes', 'rviz', 'docs'):
    for path in sorted(Path(directory).rglob('*')):
        if path.is_file() and '__pycache__' not in path.parts:
            data.append(('share/' + PACKAGE + '/' + path.parent.as_posix(), [str(path)]))
setup(
    name=PACKAGE, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=data, install_requires=['setuptools', 'PyYAML'], zip_safe=False,
    maintainer='FR3 maintainer', maintainer_email='maintainer@example.com',
    description='FR3 MoveIt 2 real commissioning', license='MIT',
    entry_points={'console_scripts': [
        'check_feedback = fr3_real_bringup.check_feedback:main',
    ]},
)
