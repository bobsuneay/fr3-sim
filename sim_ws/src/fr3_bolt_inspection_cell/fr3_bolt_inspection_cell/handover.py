"""Small testable transaction: receiver verification precedes donor opening."""
def transfer(io, first, second, object_pose, close_width, open_width, settle, verify):
    io.gripper(second, close_width)
    io.assisted_grasp(second)
    io.object_scene(object_pose, second, previous=first)
    settle()
    verify(object_pose)
    if io.grasp_owner() != second:
        raise RuntimeError('Receiver ownership not confirmed; donor stays closed')
    io.gripper(first, open_width)
