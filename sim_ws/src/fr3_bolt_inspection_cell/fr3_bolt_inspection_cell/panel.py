"""ROS/Tk panel: retry, measured joint/gripper feedback and camera preview."""
import json
import queue
import threading
import time
import tkinter as tk
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .feedback import JointFeedback
from .panel_ui import InspectionPanel


def main():
    rclpy.init()
    node = Node('inspection_panel')
    messages = queue.Queue(maxsize=20)
    lock = threading.Lock()
    frames, feedback = {}, JointFeedback()
    state = {'event': {}, 'controls': {}, 'status_time': 0., 'owner': None, 'owner_time': 0.}
    cameras = ['waist_camera', 'left_d435i', 'right_d435i', 'head_camera']

    def enqueue(text):
        try:
            messages.put_nowait(text)
        except queue.Full:
            pass

    def status(msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict) and 'phase' in data and 'detail' in data:
                with lock:
                    state['event'] = data
        except ValueError:
            enqueue(msg.data)

    node.create_subscription(String, '/inspection/status', status,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def receive(name, msg):
        with lock:
            frames[name] = msg

    def joints(msg):
        with lock:
            feedback.receive(msg, time.monotonic())

    node.create_subscription(JointState, '/joint_states', joints, qos_profile_sensor_data)
    for name in cameras:
        node.create_subscription(Image, '/'+name+'/image_raw',
            lambda msg, n=name: receive(n, msg), qos_profile_sensor_data)
    clients = {name: node.create_client(Trigger, topic) for name, topic in (
        ('start', '/inspection/start'), ('retry', '/inspection/retry_pick'),
        ('stop', '/inspection/stop'), ('status', '/inspection/get_status'),
        ('owner', '/inspection/sim/owner'))}
    requests = {}

    def request(name):
        client = clients[name]
        if not client.service_is_ready():
            enqueue('服务尚未就绪，请等待控制器和 MoveIt 启动。')
            return
        if name in requests and not requests[name][0].done():
            return
        future = client.call_async(Trigger.Request())
        requests[name] = (future, time.monotonic())

        def done(result):
            if result.cancelled():
                return
            try:
                response = result.result()
                enqueue(('已接受：' if response.success else '未执行：')+response.message)
            except Exception as exc:
                enqueue(str(exc))
        future.add_done_callback(done)

    def poll(name):
        client = clients[name]
        prior = requests.get(name)
        if prior and not prior[0].done():
            if time.monotonic()-prior[1] < 3:
                return
            prior[0].cancel()
            client.remove_pending_request(prior[0])
        if not client.service_is_ready():
            return
        future = client.call_async(Trigger.Request())
        requests[name] = (future, time.monotonic())

        def done(result):
            if result.cancelled():
                return
            try:
                response = result.result()
                if not response.success:
                    return
                if name == 'status':
                    data = json.loads(response.message)
                    if not isinstance(data, dict) or 'phase' not in data or 'detail' not in data:
                        return
                    with lock:
                        state.update(event=data, controls=data, status_time=time.monotonic())
                elif response.message in ('', 'left', 'right'):
                    with lock:
                        state.update(owner=response.message, owner_time=time.monotonic())
            except Exception as exc:
                node.get_logger().debug('Panel feedback: '+str(exc))
        future.add_done_callback(done)

    def spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    root = tk.Tk()
    ui = InspectionPanel(root, cameras, request)
    last, last_poll = None, -1.

    def update():
        nonlocal last, last_poll
        now = time.monotonic()
        if now-last_poll >= 1:
            poll('status')
            poll('owner')
            last_poll = now
        for name, (future, sent) in list(requests.items()):
            if name in ('start', 'retry', 'stop') and not future.done() and now-sent > 8:
                # Abandon only the local response; a remote task may already be
                # running. Read status and leave Stop available; never auto-resend.
                future.cancel()
                clients[name].remove_pending_request(future)
                del requests[name]
                enqueue('请求响应超时，请查看任务状态；必要时先停止运动。')
        while not messages.empty():
            ui.response.set(messages.get_nowait())
        with lock:
            msg = frames.get(ui.selected.get())
            measured = feedback.snapshot(now)
            latest = dict(state)
        ui.feedback(measured, latest['owner'] if now-latest['owner_time'] <= 2 else None)
        fresh = now-latest['status_time'] <= 3
        controls = dict(latest['controls']) if fresh else {}
        if latest['owner'] in ('left', 'right') and now-latest['owner_time'] <= 2:
            controls['can_retry'] = False
        ui.controls(controls, pending=any(
            not future.done() for name, (future, _) in requests.items() if name in ('start', 'retry')))
        event = latest['event']
        if event:
            ui.status.set(event['phase']+'\n'+event['detail']+('' if fresh else '\n任务状态连接中断/等待服务'))
        if msg is not None and msg is not last and msg.encoding in ('rgb8', 'bgr8'):
            last = msg
            try:
                pixels = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
                pixels = pixels[:, :msg.width*3].reshape(msg.height, msg.width, 3)
                if msg.encoding == 'bgr8':
                    pixels = pixels[:, :, ::-1]
                stride = max(1, (msg.width+759)//760, (msg.height+319)//320)
                pixels = pixels[::stride, ::stride]
                h, w = pixels.shape[:2]
                photo = tk.PhotoImage(data=f'P6\n{w} {h}\n255\n'.encode()+pixels.tobytes(), format='PPM')
                ui.preview.configure(image=photo, text='')
                ui.preview.image = photo
            except (ValueError, tk.TclError) as exc:
                ui.preview.configure(image='', text='相机图像格式异常：'+str(exc))
        root.after(100, update)

    root.after(100, update)
    try:
        root.mainloop()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2)
        node.destroy_node()
