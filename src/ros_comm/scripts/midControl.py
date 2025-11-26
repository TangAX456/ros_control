import rospy
import roslibpy
import tkinter as tk
from threading import Thread
from midControl_app_v1.msg import UAVState

class MidControlApp:
    def __init__(self, root, client):
        self.root = root
        self.client = client
        self.gps_data = {'latitude': 0, 'longitude': 0, 'altitude': 0}
        self.running = True

        # 创建窗口布局
        self.root.title("Mid Control")
        self.gps_label = tk.Label(root, text="GPS: Lat=0, Lon=0, Alt=0", font=("Arial", 14))
        self.gps_label.pack(pady=10)
        self.command_button = tk.Button(root, text="Send Command", command=self.open_command_window, font=("Arial", 12))
        self.command_button.pack(pady=10)

        self.status_data = "Waiting for status updates..."
        self.status_label = tk.Label(root, text=self.status_data, font=("Arial", 14))
        self.status_label.pack(pady=10)

        self.uav_state = {
            'latitude': 0.0,
            'longitude': 0.0,
            'altitude': 0.0,
            'battery_voltage': 0.0,
            'battery_percentage': 0.0
        }

        self.uav_label = tk.Label(root, text="UAV: Lat=0, Lon=0, Alt=0, Battery=0.0V (0%)", font=("Arial", 14))
        self.uav_label.pack(pady=10)

        # 创建命令话题
        self.command_topic = roslibpy.Topic(self.client, '/drone_command', 'std_msgs/String')

        # 启动无人机状态信息订阅线程
        self.uav_state_thread = Thread(target=self.subscribe_uav_state)
        self.uav_state_thread.daemon = True
        self.uav_state_thread.start()

        # 启动状态信息订阅线程
        self.status_thread = Thread(target=self.subscribe_status_updates)
        self.status_thread.daemon = True
        self.status_thread.start()

    def subscribe_uav_state(self):
        """
        订阅无人机状态信息
        """
        def uav_state_callback(msg):
            self.uav_state['latitude'] = msg.latitude
            self.uav_state['longitude'] = msg.longitude
            self.uav_state['altitude'] = msg.altitude
            self.uav_state['battery_voltage'] = msg.battery_state
            self.uav_state['battery_percentage'] = msg.battery_percetage * 100

        rospy.Subscriber('/uav1/prometheus/state', UAVState, uav_state_callback)

        while self.running:
            self.uav_label.config(
                text=f"UAV: Lat={self.uav_state['latitude']:.6f}, Lon={self.uav_state['longitude']:.6f}, "
                     f"Alt={self.uav_state['altitude']:.2f}m, Battery={self.uav_state['battery_voltage']:.2f}V "
                     f"({self.uav_state['battery_percentage']:.1f}%)"
            )
            self.root.update_idletasks()
            rospy.sleep(1)

    def subscribe_status_updates(self):
        """
        订阅状态更新话题
        """
        status_topic = roslibpy.Topic(self.client, '/uav_status_update', 'std_msgs/String')

        def status_callback(message):
            self.status_data = message.get('data', "No status received")
            self.status_label.config(text=f"Status: {self.status_data}")

        status_topic.subscribe(status_callback)

        while self.running:
            rospy.sleep(1)

        status_topic.unsubscribe()

    def open_command_window(self):
        """
        打开命令发送窗口
        """
        command_window = tk.Toplevel(self.root)
        command_window.title("Send Command")
        tk.Label(command_window, text="Enter Command:", font=("Arial", 12)).pack(pady=5)
        command_entry = tk.Entry(command_window, font=("Arial", 12))
        command_entry.pack(pady=5)
        tk.Button(
            command_window,
            text="Send",
            command=lambda: self.send_command(command_entry.get()),
            font=("Arial", 12)
        ).pack(pady=10)

    def send_command(self, command):
        """
        发送命令到无人机
        """
        rospy.loginfo(f"Sending command: {command}")
        command_message = roslibpy.Message({'data': command})
        self.command_topic.publish(command_message)
        rospy.loginfo(f"Command sent: {command}")
        # 无需取消话题广告，保持话题可用

    def stop(self):
        """
        停止应用程序
        """
        self.running = False
        self.status_thread.join()

def main():
    rospy.init_node('mid_control', anonymous=True)

    # 从参数获取连接信息
    bridge_host = '192.168.1.11'  # 无人机 IP
    bridge_port = 9090

    rospy.loginfo(f"Connecting to rosbridge at {bridge_host}:{bridge_port}")
    try:
        client = roslibpy.Ros(host=bridge_host, port=bridge_port)
        client.run()
        rospy.loginfo("Connected to rosbridge server")

        # 启动 Tkinter 窗口
        root = tk.Tk()
        app = MidControlApp(root, client)
        root.protocol("WM_DELETE_WINDOW", app.stop)  # 窗口关闭时停止程序
        root.mainloop()

    except Exception as e:
        rospy.logerr(f"Error: {e}")
    finally:
        if 'client' in locals():
            client.terminate()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
