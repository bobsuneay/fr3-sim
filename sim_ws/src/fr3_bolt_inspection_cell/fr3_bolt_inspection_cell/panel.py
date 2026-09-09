"""Small ROS/Tk operator panel: start, cancel, status and live camera preview."""
import json
import queue
import threading
import tkinter as tk
from tkinter import ttk
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = Node('inspection_panel')
    messages = queue.Queue(maxsize=10)
    frame_lock = threading.Lock()
    frames = {}
    def enqueue(text):
        try:
            messages.put_nowait(text)
        except queue.Full:
            pass
    def status(msg):
        try:
            data = json.loads(msg.data)
            enqueue(data['phase']+'\n'+data['detail'])
        except (ValueError, KeyError):
            enqueue(msg.data)
    node.create_subscription(String, '/inspection/status', status,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    cameras = ['waist_camera', 'left_d435i', 'right_d435i', 'head_camera']
    def receive(name, msg):
        with frame_lock:
            frames[name] = msg
    for name in cameras:
        node.create_subscription(Image, '/'+name+'/image_raw',
            lambda msg, n=name: receive(n, msg), qos_profile_sensor_data)
    start = node.create_client(Trigger, '/inspection/start')
    stop = node.create_client(Trigger, '/inspection/stop')
    def spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    root = tk.Tk()
    root.title('FR3 双臂螺丝检测 · Gazebo')
    root.geometry('800x630')
    ttk.Label(root, text='点云定位 → 抓取 → 定点转动拍摄 → 双臂交接 → 再次拍摄',
              font=('', 13)).pack(pady=12)
    bar = ttk.Frame(root)
    bar.pack()
    def request(client):
        if not client.service_is_ready():
            enqueue('服务尚未就绪，请等待控制器和 MoveIt 启动。')
            return
        def done(future):
            try:
                response = future.result()
                enqueue(('已接受：' if response.success else '未启动：')+response.message)
            except Exception as exc:
                enqueue(str(exc))
        client.call_async(Trigger.Request()).add_done_callback(done)
    ttk.Button(bar, text='开始单次完整任务', command=lambda: request(start)).pack(side='left', padx=10)
    ttk.Button(bar, text='停止运动并保持夹持', command=lambda: request(stop)).pack(side='left', padx=10)
    selected = tk.StringVar(value=cameras[0])
    ttk.Combobox(bar, textvariable=selected, values=cameras, state='readonly', width=18).pack(side='left')
    text = tk.StringVar(value='等待 /inspection/status；启动按钮需要 enable_execution:=true')
    ttk.Label(root, textvariable=text, wraplength=760, justify='left').pack(padx=20, pady=12)
    preview = ttk.Label(root, text='等待相机图像')
    preview.pack(fill='both', expand=True)
    ttk.Label(root, text='仿真使用几何检查后的固定关节夹持辅助。关闭面板不会停止后台任务。').pack(pady=8)
    last = None
    def update():
        nonlocal last
        while not messages.empty():
            text.set(messages.get_nowait())
        with frame_lock:
            msg = frames.get(selected.get())
        if msg is not None and msg is not last and msg.encoding in ('rgb8', 'bgr8'):
            last = msg
            pixels = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
            pixels = pixels[:, :msg.width*3].reshape(msg.height, msg.width, 3)
            if msg.encoding == 'bgr8':
                pixels = pixels[:, :, ::-1]
            stride = max(1, (msg.width+759)//760)
            pixels = pixels[::stride, ::stride]
            h, w = pixels.shape[:2]
            photo = tk.PhotoImage(data=f'P6\n{w} {h}\n255\n'.encode()+pixels.tobytes(), format='PPM')
            preview.configure(image=photo, text='')
            preview.image = photo
        root.after(100, update)
    root.after(100, update)
    try:
        root.mainloop()
    finally:
        rclpy.shutdown()
        thread.join(timeout=2)
        node.destroy_node()
