from pathlib import Path
from setuptools import find_packages, setup

name = 'fr3_dual_bolt_cell'
data = [('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml', 'README.md', 'LICENSE', 'THIRD_PARTY.md'])]
for directory in ('config', 'urdf', 'launch', 'rviz', 'meshes', 'docs', 'tools'):
    for path in sorted(Path(directory).rglob('*')):
        if path.is_file() and '__pycache__' not in path.parts:
            data.append(('share/' + name + '/' + path.parent.as_posix(), [str(path)]))
setup(name=name, version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=data, install_requires=['setuptools', 'PyYAML'], zip_safe=False,
      maintainer='FR3 cell maintainer', maintainer_email='maintainer@example.com',
      description='Dual FAIRINO FR3 bolt cell with Gazebo, mock and real backends',
      license='MIT', entry_points={'console_scripts': [
          'publish_scene = fr3_dual_bolt_cell.planning_scene:main',
          'gripper = fr3_dual_bolt_cell.gripper:main',
          'check_feedback = fr3_dual_bolt_cell.check_feedback:main',
      ]})
