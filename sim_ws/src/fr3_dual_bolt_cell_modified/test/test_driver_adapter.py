"""Set FR3_VENDOR_DRIVER to the supplied 3.9.7 package to exercise copy+patch."""
import importlib.util
import os
from pathlib import Path

import pytest

SHARE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('prepare_driver', SHARE/'tools/prepare_driver.py')
adapter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)


def test_different_driver_source_rejected(tmp_path):
    source = tmp_path/'source'
    (source/'src').mkdir(parents=True)
    (source/'src/fairino_hardware_interface.cpp').write_text('wrong revision')
    with pytest.raises(ValueError): adapter.prepare(source, tmp_path/'output')
    assert not (tmp_path/'output').exists()


def test_existing_destination_not_overwritten(tmp_path):
    destination = tmp_path/'existing'
    destination.mkdir()
    with pytest.raises(FileExistsError): adapter.prepare(tmp_path/'absent', destination)


@pytest.mark.skipif(not os.environ.get('FR3_VENDOR_DRIVER'), reason='External vendor source not supplied')
def test_actual_source_copy_patch_and_original_preserved(tmp_path):
    source = Path(os.environ['FR3_VENDOR_DRIVER'])
    original = (source/'src/fairino_hardware_interface.cpp').read_bytes()
    destination = adapter.prepare(source, tmp_path/'adapted')
    output = (destination/'src/fairino_hardware_interface.cpp').read_text(encoding='utf-8')
    assert (source/'src/fairino_hardware_interface.cpp').read_bytes() == original
    assert '_controller_ip = ip->second;' in output
    assert 'info_.joints.size() != 6' in output
    assert '&_jnt_position_command[6]' in output
    assert '&_jnt_torque_command[6]' in output
    assert '        hardware_interface::return_type::ERROR;' not in output
    assert '_jnt_position_state[j] = _jnt_position_command[j];' in output
    assert '_ptr_robot.release()' not in output
    assert (destination/'dual_cell_adapter.diff').is_file()
    assert (destination/'dual_cell_adapter_v1.txt').is_file()
    assert 'install(FILES dual_cell_adapter_v1.txt' in (destination/'CMakeLists.txt').read_text()
