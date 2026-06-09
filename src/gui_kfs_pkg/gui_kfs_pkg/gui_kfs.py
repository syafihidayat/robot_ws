import ttkbootstrap as tb
from ttkbootstrap.constants import *

import rclpy
import rclpy.signals
from rclpy.node import Node
from std_msgs.msg import Bool
from mehua_pkg_msgs.msg import KFSDetectionArray
from rclpy.qos import QoSProfile, DurabilityPolicy


class GridGUI(Node):
    def __init__(self):
        super().__init__('grid_gui_node')

        self.has_kfs = False
        self.latest_detection = None

        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.decision_pub = self.create_publisher(Bool, "/robot_start333333333", qos)

        self.button_retryStage3 = self.create_publisher(Bool, "/button_stage3", 10)

        self.button_retryStage2 = self.create_publisher(Bool, "/button_stage2", 10)

        self.detection_yolo_sub = self.create_subscription(
            KFSDetectionArray, "/kfs_detections", self.KFSDetection_callback, 10
        )

        self.root = tb.Window(themename="cosmo")
        self.root.title("Robot GUI")
        self.root.geometry("360x250")

        self.build_gui()
        self.root.after(100, self.ros_spin)

    def ros_spin(self):
        rclpy.spin_once(self, timeout_sec=0)
        self.root.after(50, self.ros_spin)

    def build_gui(self):
        tb.Label(
            self.root,
            text=" Robot Control",
            font=("Arial", 14, "bold")
        ).pack(pady=(15, 10))

        btn_opts = dict(width=20, padding=(0, 12))

        tb.Button(
            self.root,
            text="▶  START",
            bootstyle="success",
            command=self.send_start,
            **btn_opts
        ).pack(pady=5)
        
        tb.Button(
            self.root,
            text="↺  RETRY STAGE 2",
            bootstyle="warning",
            command=self.send_retry_stage2,
            **btn_opts
        ).pack(pady=5)

        tb.Button(
            self.root,
            text="↺  RETRY STAGE 3",
            bootstyle="warning",
            command=self.send_retry_stage3,
            **btn_opts
        ).pack(pady=5)


    def send_start(self):
        msg = Bool()
        msg.data = True
        self.decision_pub.publish(msg)
        print("START sent: True")

    def send_retry_stage3(self):
        msg = Bool()
        msg.data = True
        self.button_retryStage3.publish(msg)
        print("RETRY STAGE 3 sent: True")

    def send_retry_stage2(self):
        msg = Bool()
        msg.data = True
        self.button_retryStage2.publish(msg)
        print("RETRY STAGE 2 sent: True")

    def KFSDetection_callback(self, msg: KFSDetectionArray):
        if not msg.detections:
            self.has_kfs = False
            return
        det = max(msg.detections, key=lambda d: d.confidence)
        self.latest_detection = det
        self.has_kfs = True

    def run(self):
        self.root.mainloop()


def main():
    # rclpy.init()
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    
    gui_node = GridGUI()
    try:
        gui_node.run()
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()