#!/usr/bin/env python3
"""Copy the supplied 3.9.7 driver to a NEW folder, then apply a bounded adapter.

No robot connection, build or modification of the source directory is performed.
The generated unified diff is the review record. Firmware selection remains local.
"""
import argparse
import difflib
import hashlib
from pathlib import Path
import shutil

SOURCE_HASH = '6f09a14042faa8fb924dd6be495b81958e5f26d604599a1c113d3c72b6a344c1'


def replace_once(text, before, after):
    if text.count(before) != 1:
        raise ValueError('Unexpected driver revision at anchor: '+before[:80])
    return text.replace(before, after, 1)


def adapt(source):
    # Normalise CRLF so a Linux checkout and Windows archive behave identically.
    text = source.replace('\r\n', '\n')
    insertion = '''
    // dual_cell_adapter_v1: fail closed, never fall back to the compiled IP.
    if (info_.joints.size() != 6) {
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "Exactly six arm joints required");
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto ip = info_.hardware_parameters.find("robot_ip");
    if (ip == info_.hardware_parameters.end() || ip->second.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("FairinoHardwareInterface"), "robot_ip is required");
        return hardware_interface::CallbackReturn::ERROR;
    }
    _controller_ip = ip->second;
'''
    text = replace_once(text, '    info_ = sysinfo;', '    info_ = sysinfo;'+insertion)
    text = replace_once(text, '            _jnt_position_command[j] = jntpos.jPos[j]/180.0*M_PI;',
        '''            if (!std::isfinite(jntpos.jPos[j])) {
                return hardware_interface::CallbackReturn::ERROR;
            }
            _jnt_position_command[j] = jntpos.jPos[j]/180.0*M_PI;
            _jnt_position_state[j] = _jnt_position_command[j];''')
    text = replace_once(text, '        hardware_interface::return_type::ERROR;',
                        '        return hardware_interface::return_type::ERROR;')
    text = replace_once(text, '            _jnt_position_state[i] = state_data.jPos[i]/180.0*M_PI;',
        '''            if (!std::isfinite(state_data.jPos[i])) {
                return hardware_interface::return_type::ERROR;
            }
            _jnt_position_state[i] = state_data.jPos[i]/180.0*M_PI;''')
    for kind in ('position', 'torque'):
        text = replace_once(text, f'&_jnt_{kind}_command[5]', f'&_jnt_{kind}_command[6]')
    anchor = '"ServoJ指令下发错误,错误码:%d",returncode);'
    text = replace_once(text, anchor, anchor+'\n            return hardware_interface::return_type::ERROR;')
    text = replace_once(text, '_ptr_robot.release();', '_ptr_robot.reset();')
    return text


def prepare(source, destination):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if source == destination or source in destination.parents or destination in source.parents:
        raise ValueError('Destination must be outside source and must not contain source')
    if destination.exists():
        raise FileExistsError('Destination already exists; choose a new folder')
    file = source/'src/fairino_hardware_interface.cpp'
    raw = file.read_bytes()
    # Verify both archive bytes and newline-normalised form for git autocrlf.
    normal = raw.replace(b'\r\n', b'\n')
    hashes = {hashlib.sha256(raw).hexdigest(), hashlib.sha256(normal).hexdigest(),
              hashlib.sha256(normal.replace(b'\n', b'\r\n')).hexdigest()}
    if SOURCE_HASH not in hashes:
        raise ValueError('Source hash differs from the supplied 3.9.7 implementation; review manually')
    before = raw.decode('utf-8').replace('\r\n', '\n')
    after = adapt(before)
    cmake_before = (source/'CMakeLists.txt').read_text(encoding='utf-8')
    cmake = replace_once(cmake_before, 'ament_package()',
        'install(FILES dual_cell_adapter_v1.txt DESTINATION share/${PROJECT_NAME})\nament_package()')
    shutil.copytree(source, destination)
    (destination/'src/fairino_hardware_interface.cpp').write_text(after, encoding='utf-8')
    (destination/'CMakeLists.txt').write_text(cmake, encoding='utf-8')
    (destination/'dual_cell_adapter_v1.txt').write_text(
        'Source SHA256: '+SOURCE_HASH+'\nrobot_ip parameter; isolated manager per arm; 125 Hz ServoJ\n',
        encoding='utf-8')
    diff = ''.join(difflib.unified_diff(
        before.splitlines(True), after.splitlines(True),
        fromfile='original/src/fairino_hardware_interface.cpp',
        tofile='adapted/src/fairino_hardware_interface.cpp'))
    diff += ''.join(difflib.unified_diff(cmake_before.splitlines(True), cmake.splitlines(True),
                   fromfile='original/CMakeLists.txt', tofile='adapted/CMakeLists.txt'))
    (destination/'dual_cell_adapter.diff').write_text(diff, encoding='utf-8')
    return destination


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path, help='fairino_hardware_v3_9_7 directory')
    parser.add_argument('--destination', required=True, type=Path, help='New package directory')
    args = parser.parse_args()
    print(prepare(args.source, args.destination))
