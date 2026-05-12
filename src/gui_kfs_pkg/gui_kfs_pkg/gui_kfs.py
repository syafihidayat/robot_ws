import ttkbootstrap as tb
from ttkbootstrap.constants import *
from tkinter import END, messagebox

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32MultiArray
from gui_kfs_msgs.msg import KFSDecision
from mehua_pkg_msgs.msg import KFSDetectionArray
from std_msgs.msg import Bool

GRID_ROWS = 4
GRID_COLS = 3

class GridGUI(Node):
    def __init__(self):
        super().__init__('grid_gui_node')
        
        # posisi robot dalam indeks asli (0..11, baris-mayor)
        self.robot_pos = {"KFS R1": [], "KFS R2": [], "KFS FAKE": []}
        self.active_robot = "KFS R1"

        self.decision_pub = self.create_publisher(KFSDecision, "/kfs_decision", 10)
        self.goal_reached_sub = self.create_subscription(Bool,"/goal_reached", self.goal_reached_callback, 10 )
        self.detection_yolo_sub = self.create_subscription(KFSDetectionArray, "/kfs_detections", self.KFSDetection_callback, 10)

        # Window
        self.root = tb.Window(themename="cosmo")
        self.root.title("Robot Grid Dashboard")
        self.root.geometry("640x640")

        self.buttons = []  # list tombol sesuai indeks asli

        self.has_kfs = False
        self.latest_detection = None

        self.build_gui()
        self.update_decision()

        self.root.after(100, self.ros_spin)
        self.root.after(50, self.update_from_yolo)

        self.robot_actual_pos = None
        self.chase_sent = []

    def ros_spin(self):
        rclpy.spin_once(self, timeout_sec=0)
        self.root.after(50, self.ros_spin)

    def build_gui(self):
        # Top frame untuk robot select + counter
        top = tb.Frame(self.root)
        top.pack(pady=15, fill=X)

        tb.Button(
            self.root,
            text="Kirim Target",
            bootstyle="success",
            padding=(15,8),
            command=self.send_decision
        ).pack(pady=5)

        self.robot_buttons = {}
        self.robot_counters = {}
        robot_color = {"KFS R1":"danger", "KFS R2":"info", "KFS FAKE":"warning"}

        for r in ["KFS R1", "KFS R2", "KFS FAKE"]:
            frame = tb.Frame(top)
            frame.pack(side=LEFT, padx=15)

            btn = tb.Button(frame, text=r, bootstyle=robot_color[r],
                            padding=(20,12),
                            command=lambda x=r: self.select_robot(x))
            btn.pack()

            counter = tb.Label(frame, text="0", font=("Arial", 10))
            counter.pack(pady=3)

            self.robot_buttons[r] = btn
            self.robot_counters[r] = counter

        # Grid frame - tanpa mirroring, indeks asli 0..11
        grid_frame = tb.Frame(self.root)
        grid_frame.pack(pady=20)

        self.total_cells = GRID_ROWS * GRID_COLS
        for idx in range(self.total_cells):
            btn = tb.Button(grid_frame,
                            text=str(idx),
                            bootstyle="secondary",
                            padding=(25,25),
                            command=lambda i=idx: self.toggle_robot(i))
            btn.grid(row=idx // GRID_COLS, column=idx % GRID_COLS, padx=8, pady=8)
            self.buttons.append(btn)

        # Clear All button
        tb.Button(self.root, text="Hapus Semua", bootstyle="danger",
                  padding=(15,8), command=self.clear_all).pack(pady=10)

        # Decision box
        self.decision_box = tb.Text(self.root, height=6, width=60)
        self.decision_box.pack(pady=10)
        self.decision_box.insert(END, "Menunggu semua KFS ditempatkan...")
        self.decision_box.configure(state='disabled')

    def select_robot(self, name):
        self.active_robot = name
        robot_color = {"KFS R1":"danger", "KFS R2":"info", "KFS FAKE":"warning"}
        for r, btn in self.robot_buttons.items():
            if r == name:
                btn.configure(bootstyle=robot_color[r])
            else:
                btn.configure(bootstyle="secondary")

    def goal_reached_callback(self, msg):
        if msg.data:
            self.send_stop()

    def send_stop(self):
        stop_msg = KFSDecision()
        stop_msg.mode = 0
        stop_msg.chase_targets = []
        stop_msg.avoid_targets = []
        self.decision_pub.publish(stop_msg)

        self.decision_box.configure(state='normal')
        self.decision_box.delete('1.0', END)
        self.decision_box.insert(END, "✅ ROBOT SAMPAI — STOP dikirim otomatis")
        self.decision_box.configure(state='disabled')
        print("Auto-stop sent: goal reached")

    def KFSDetection_callback(self, msg: KFSDetectionArray):
        if not msg.detections:
            self.has_kfs = False
            return
        det = max(msg.detections, key=lambda d: d.confidence)
        self.latest_detection = det
        self.has_kfs = True

    def update_from_yolo(self):
        if self.has_kfs and self.latest_detection is not None:
            det = self.latest_detection
            # jika perlu menampilkan marker KFS di grid, implementasikan di sini
            # self.update_kfs_marker(det.x, det.y, det.kfs_type)
            pass
        self.root.after(50, self.update_from_yolo)

    def toggle_robot(self, index):
        """index adalah indeks asli tombol (0..11)"""
        # cek apakah index sudah ditempati robot apa pun
        found = None
        for r, poses in self.robot_pos.items():
            if index in poses:
                found = r
                break

        if found:
            self.robot_pos[found].remove(index)
        else:
            limits = {"KFS R1": 3, "KFS R2": 4, "KFS FAKE": 1}
            if len(self.robot_pos[self.active_robot]) >= limits[self.active_robot]:
                messagebox.showwarning("Limit", f"{self.active_robot} sudah maksimal!")
                return
            self.robot_pos[self.active_robot].append(index)

        self.render()
        self.update_decision()

    def render(self):
        # Reset semua tombol ke tampilan indeks asli
        for idx, btn in enumerate(self.buttons):
            btn.configure(text=str(idx), bootstyle="secondary")

        robot_color = {"KFS R1":"danger", "KFS R2":"info", "KFS FAKE":"warning"}

        # Gambar robot di grid
        for r, poses in self.robot_pos.items():
            for p in poses:
                self.buttons[p].configure(text=r, bootstyle=robot_color[r])

        # Update counter
        limits = {"KFS R1": 3, "KFS R2": 4, "KFS FAKE": 1}
        for r, counter in self.robot_counters.items():
            counter.configure(text=f"{len(self.robot_pos[r])}/{limits[r]}")

    def clear_all(self):
        for key in self.robot_pos:
            self.robot_pos[key] = []
        self.render()
        self.update_decision()

    def process_target(self):
        r2_positions = self.robot_pos["KFS R2"]
        chase_raw = r2_positions[:2]                     # indeks asli
        avoid_from_r2 = r2_positions[2:]
        avoid_all = self.robot_pos["KFS R1"] + self.robot_pos["KFS FAKE"] + avoid_from_r2

        has_chase = len(chase_raw) > 0
        has_avoid = len(avoid_all) > 0

        if has_chase and has_avoid:
            mode = 3
        elif has_chase:
            mode = 1
        elif has_avoid:
            mode = 2
        else:
            mode = 0

        return mode, chase_raw, avoid_all   # langsung indeks asli

    def send_decision(self):
        mode, chase_indices, avoid_indices = self.process_target()

        if not (chase_indices or avoid_indices):
            messagebox.showwarning("Warning", "Belum ada target!")
            return

        msg = KFSDecision()
        msg.mode = mode
        msg.chase_targets = chase_indices   # kirim indeks asli
        msg.avoid_targets = avoid_indices

        self.decision_pub.publish(msg)

        decision = {
            "mode": mode,
            "chase": chase_indices,
            "avoid": avoid_indices
        }

        self.decision_box.configure(state='normal')
        self.decision_box.delete('1.0', END)
        self.decision_box.insert(END, "TARGET TERKIRIM\n")
        self.decision_box.insert(END, str(decision))
        self.decision_box.configure(state='disabled')

        print("Decision sent:", decision)

    def update_decision(self):
        self.decision_box.configure(state='normal')
        self.decision_box.delete('1.0', END)

        mode, chase_indices, avoid_indices = self.process_target()

        if not (chase_indices or avoid_indices):
            self.decision_box.insert(END, "Menunggu KFS dipilih...")
            self.decision_box.configure(state="disabled")
            return

        decision = {
            "mode": mode,
            "chase": chase_indices,
            "avoid": avoid_indices
        }

        self.decision_box.insert(END, "Preview Decision:\n")
        self.decision_box.insert(END, str(decision))
        self.decision_box.configure(state='disabled')

    def run(self):
        self.root.mainloop()

def main():
    rclpy.init()
    gui_node = GridGUI()
    try:
        gui_node.run()
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()