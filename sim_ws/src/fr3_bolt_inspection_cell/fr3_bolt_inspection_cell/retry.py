"""Recovery for a failed initial pick; never release a held object."""


def prepare_pick_retry(io, first, arms, cfg, target, settle):
    owner = io.grasp_owner()
    if owner:
        raise RuntimeError(f'{owner} already holds the object; retry will not open the jaws')
    io.wait_stationary()
    # Ownership may have changed while cancellation was settling.
    if io.grasp_owner():
        raise RuntimeError('Object acquired while waiting; retry will not open the jaws')
    second = 'left' if first == 'right' else 'right'
    io.gripper(first, cfg['open_width'])
    io.gripper(second, cfg['open_width'])
    settle()
    if target is not None:
        actual = io.tcp_pose(first)
        clearance = target[2, 3]+cfg['approach_height']
        if actual[2, 3] < clearance:
            above = actual.copy()
            above[2, 3] = clearance
            io.cartesian(first, [above], cfg['descent_speed'])
    for side in (first, second):
        io.global_move(side, joints=arms[side]['initial'])
    settle()
    io.allow_touch([s+'_'+f+'_finger' for s in ('left', 'right')
                    for f in ('left', 'right')], False)
