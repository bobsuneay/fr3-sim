"""JointState storage/formatting shared by the panel and offline tests."""
import math


class JointFeedback:
    def __init__(self):
        self.joints = {}

    def receive(self, msg, now):
        if len(msg.name) != len(msg.position):
            return
        for name, value in zip(msg.name, msg.position):
            if math.isfinite(value):
                self.joints[name] = (float(value), now)
            else:
                self.joints.pop(name, None)

    def snapshot(self, now, max_age=2.0):
        return {name: value for name, (value, received) in self.joints.items()
                if 0 <= now-received <= max_age}


def arm_rows(joints):
    rows = []
    for i in range(1, 7):
        row = [f'J{i}']
        for side in ('left', 'right'):
            value = joints.get(f'{side}_j{i}')
            row.extend(('--', '--', '无数据/过期') if value is None else
                       (f'{value:.4f}', f'{math.degrees(value):.2f}', '实时'))
        rows.append(row)
    return rows


def gripper_text(side, joints, owner):
    values = [joints.get(f'{side}_{finger}_finger_joint') for finger in ('left', 'right')]
    positions = ['--' if v is None else f'{1000*v:.2f}' for v in values]
    gap = '--' if any(v is None for v in values) else f'{1000*sum(values):.2f}'
    holding = '未知/反馈过期' if owner is None else ('夹持中（仿真辅助）' if owner == side else '未夹持')
    return (f'左指：{positions[0]} mm    右指：{positions[1]} mm\n'
            f'开口宽度：{gap} mm\n夹持反馈：{holding}')
