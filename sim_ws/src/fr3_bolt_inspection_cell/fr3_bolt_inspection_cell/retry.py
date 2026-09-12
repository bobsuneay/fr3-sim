"""Recovery for a failed initial pick; never release a held object."""


def prepare_pick_retry(io, first, arms, cfg, target, settle):
    """Local retreat from measured pose; saved home/old target are not used."""
    owner = io.grasp_owner()
    if owner:
        raise RuntimeError(f'{owner} already holds the object; retry will not open the jaws')
    io.wait_stationary()
    # Ownership may have changed while cancellation was settling.
    if io.grasp_owner():
        raise RuntimeError('Object acquired while waiting; retry will not open the jaws')
    io.gripper(first, cfg['open_width'])
    settle()
    actual = io.tcp_pose(first)
    above = actual.copy()
    above[2, 3] += cfg['retry_lift_height']
    io.cartesian(first, [above], cfg['descent_speed'])
    settle()
    io.allow_touch([s+'_'+f+'_finger' for s in ('left', 'right')
                    for f in ('left', 'right')], False)
