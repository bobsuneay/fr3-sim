"""Tk widgets, independent of ROS so the operator layout can be checked offline."""
import tkinter as tk
from tkinter import ttk
from .feedback import arm_rows, gripper_text


class InspectionPanel:
    def __init__(self, root, cameras, request):
        root.title('FR3 双臂螺丝检测 · Gazebo')
        root.geometry('980x860')
        root.minsize(900, 780)
        ttk.Label(root, text='点云定位 → 抓取 → 定点转动拍摄 → 双臂交接 → 再次拍摄',
                  font=('', 13)).pack(pady=10)
        bar = ttk.Frame(root)
        bar.pack(fill='x', padx=18)
        self.start = ttk.Button(bar, text='开始完整任务', command=lambda: request('start'), state='disabled')
        self.start.pack(side='left', padx=4)
        self.retry = ttk.Button(bar, text='重新夹取', command=lambda: request('retry'), state='disabled')
        self.retry.pack(side='left', padx=4)
        self.randomize = ttk.Button(bar, text='随机零件位置',
                                    command=lambda: request('randomize'), state='disabled')
        self.randomize.pack(side='left', padx=4)
        self.handover = ttk.Button(bar, text='跳到左手交接',
                                   command=lambda: request('handover'), state='disabled')
        self.handover.pack(side='left', padx=4)
        ttk.Button(bar, text='停止运动并保持夹持', command=lambda: request('stop')).pack(side='left', padx=4)
        self.selected = tk.StringVar(value=cameras[0])
        ttk.Combobox(bar, textvariable=self.selected, values=cameras,
                     state='readonly', width=18).pack(side='right')
        self.status = tk.StringVar(value='等待任务状态；启动需要 enable_execution:=true')
        ttk.Label(root, textvariable=self.status, wraplength=920, justify='left').pack(fill='x', padx=22, pady=(10, 4))
        self.response = tk.StringVar(value='夹取失败后可重试；重新定位并夹取，成功后继续检测流程。')
        ttk.Label(root, textvariable=self.response, wraplength=920).pack(fill='x', padx=22, pady=(0, 8))
        data = ttk.LabelFrame(root, text='双臂实时关节反馈 · 10 Hz 刷新 · 缺失或超过 2 秒显示无数据')
        data.pack(fill='x', padx=18)
        columns = ('joint', 'left_rad', 'left_deg', 'left_state', 'right_rad', 'right_deg', 'right_state')
        self.table = ttk.Treeview(data, columns=columns, show='headings', height=6, selectmode='none')
        for name, title in zip(columns, ('关节', '左臂 rad', '左臂 °', '左臂反馈', '右臂 rad', '右臂 °', '右臂反馈')):
            self.table.heading(name, text=title)
            self.table.column(name, width=120 if name != 'joint' else 55, anchor='center')
        self.table.pack(fill='x', padx=6, pady=6)
        for row in arm_rows({}):
            self.table.insert('', 'end', iid=row[0], values=row)
        grippers = ttk.Frame(root)
        grippers.pack(fill='x', padx=18, pady=8)
        self.grippers = {}
        for side, title in (('left', '左臂夹爪'), ('right', '右臂夹爪')):
            box = ttk.LabelFrame(grippers, text=title)
            box.pack(side='left', fill='x', expand=True, padx=4)
            self.grippers[side] = tk.StringVar(value=gripper_text(side, {}, None))
            ttk.Label(box, textvariable=self.grippers[side], justify='left').pack(anchor='w', padx=10, pady=6)
        self.preview = ttk.Label(root, text='等待相机图像', anchor='center')
        self.preview.pack(fill='both', expand=True, padx=18)
        ttk.Label(root, text='开口宽度来自两个夹指关节之和；夹持反馈来自仿真辅助夹持器。关闭面板不会停止任务。').pack(pady=8)

    def feedback(self, joints, owner):
        for row in arm_rows(joints):
            self.table.item(row[0], values=row)
        for side in self.grippers:
            self.grippers[side].set(gripper_text(side, joints, owner))

    def controls(self, state, pending=False):
        self.handover.configure(state='normal' if state.get('can_handover') and not pending else 'disabled')
        self.start.configure(state='normal' if state.get('can_start') and not pending else 'disabled')
        self.retry.configure(state='normal' if state.get('can_retry') and not pending else 'disabled')
        self.randomize.configure(
            state='normal' if state.get('can_randomize') and not pending else 'disabled')
