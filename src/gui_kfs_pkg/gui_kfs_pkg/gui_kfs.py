import ttkbootstrap as tb
from ttkbootstrap.constants import *
from tkinter import END,messagebox

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int32MultiArray
from gui_kfs_msgs.msg import KFSDecision
from mehua_pkg_msgs.msg import KFSDetectionArray

GRID_ROWS = 4
GRID_COLS = 3

class GridGUI(Node):
    def __init__(self):

        super().__init__('grid_gui_node')
        
        # posisi robot
        self.robot_pos = {"KFS R1": [], "KFS R2": [], "KFS FAKE": []}
        self.active_robot = "KFS R1"

        # self.chase_pub = self.create_publisher(Int32MultiArray, 'chase_from_backend',10)
        # self.avoid_pub = self.create_publisher(Int32MultiArray, 'avoid_from_backend',10)

        self.decision_pub = self.create_publisher(KFSDecision, "/kfs_decision", 10)
        self.detection_yolo_sub = self.create_subscription(KFSDetectionArray, "/kfs_detections",self.KFSDetection_callback, 10)

        # Window ttkbootstrap
        self.root = tb.Window(themename="cosmo")
        self.root.title("Robot Grid Dashboard")
        self.root.geometry("640x640")

        self.buttons = []
        self.display_to_index = {}  # mapping display_number → index asli

        self.has_kfs = False
        self.latest_detection = None

        self.build_gui()
        self.update_decision()

        self.root.after(100,self.ros_spin)
        self.root.after(50, self.update_from_yolo)

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

        # Grid frame
        grid_frame = tb.Frame(self.root)
        grid_frame.pack(pady=20)

        self.total_cells = GRID_ROWS * GRID_COLS
        for i in range(self.total_cells):
            display_number = self.total_cells - 1 - i
            btn = tb.Button(grid_frame,
                            text=str(display_number),
                            bootstyle="secondary",
                            padding=(25,25),
                            command=lambda x=display_number: self.toggle_robot(x))
            btn.grid(row=i//GRID_COLS, column=i%GRID_COLS, padx=8, pady=8)
            self.buttons.append(btn)
            self.display_to_index[display_number] = i

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
            self.update_kfs_marker(det.x,det.y,det.kfs_type)

        self.root.after(50,self.update_from_yolo)

    def toggle_robot(self, display_number):
        """Jika cell kosong, tempatkan robot, jika sudah ada robot, hapus robot."""
        index = self.display_to_index[display_number]                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       

        # cek apakah index sudah ditempati robot apa pun
        found = None
        for r, poses in self.robot_pos.items():
            if index in poses:
                found = r
                break

        if found:
            # hapus robot di sel itu
            self.robot_pos[found].remove(index)
        else:
            # tambahkan robot aktif
            limits = {"KFS R1": 3, "KFS R2": 4, "KFS FAKE": 1}
            if len(self.robot_pos[self.active_robot]) >= limits[self.active_robot]:
                # tb.messagebox.showwarning("Limit", f"{self.active_robot} sudah maksimal!")
                messagebox.showwarning("Limit", f"{self.active_robot} sudah maksimal!")
                return
            self.robot_pos[self.active_robot].append(index)

        self.render()
        self.update_decision()

    def render(self):
        # Reset grid
        for display_number, index in self.display_to_index.items():
            self.buttons[index].configure(text=str(display_number), bootstyle="secondary")

        robot_color = {"KFS R1":"danger", "KFS R2":"info", "KFS FAKE":"warning"}

        # Render robot di grid
        for r, poses in self.robot_pos.items():
            for p in poses:
                self.buttons[p].configure(text=r, bootstyle=robot_color[r])

        # Update counters
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

        chase_raw = r2_positions[:2]

        avoid_from_r2 = r2_positions[2:]

        avoid_all = (self.robot_pos["KFS R1"] + self.robot_pos["KFS FAKE"] + avoid_from_r2)

        display_chase = [self.total_cells - 1 - i for i in chase_raw]
        display_avoid = [self.total_cells - 1 - i for i in avoid_all]


        has_chase = len(display_chase) > 0
        has_avoid = len(display_avoid) > 0

        if has_chase and has_avoid:
            mode = 3
        elif has_chase:
            mode = 1
        elif has_avoid:
            mode = 2
        else:
            mode = 0

        return mode, display_chase,display_avoid
    
    def send_decision(self):
        mode, display_chase,display_avoid = self.process_target()
        # has_chase = len(self.robot_pos["KFS R2"]) > 0
        # has_avoid = len(self.robot_pos["KFS R1"]) > 0 or len(self.robot_pos["KFS FAKE"]) > 0

        if not (display_chase or display_avoid):
            messagebox.showwarning("Warning", "Belum ada target!")
            return
        
        # display_chase = [self.total_cells - 1 - i for i in self.robot_pos["KFS R2"]]
        # display_avoid = [self.total_cells - 1 - i for i in self.robot_pos["KFS R1"] + self.robot_pos["KFS FAKE"]]

        # if has_chase and has_avoid:
        #     mode = 3
        # elif has_chase:
        #     mode = 1
        # elif has_avoid:
        #     mode = 2
        # else:
        #     mode = 0

        msg = KFSDecision()
        msg.mode = mode
        msg.chase_targets = display_chase
        msg.avoid_targets = display_avoid

        self.decision_pub.publish(msg)

        decision = {
            "mode" : mode,
            "chase" : display_chase,
            "avoid" : display_avoid
        }

        self.decision_box.configure(state='normal')
        self.decision_box.delete('1.0', END)
        self.decision_box.insert(END, "TARGET TERKIRIM\n")
        self.decision_box.insert(END, str(decision))
        self.decision_box.configure(state='disabled')

        print("Decision sent:", decision)

    def update_decision(self):
        self.decision_box.configure(state='normal')
        self.decision_box.delete('1.0',END)

        # has_chase = len(self.robot_pos["KFS R2"]) > 0
        # has_avoid = len(self.robot_pos["KFS R1"]) > 0 or len(self.robot_pos["KFS FAKE"]) > 0

        mode, display_chase,display_avoid = self.process_target()

        if not (display_chase or display_avoid):
            self.decision_box.insert(END, "Menunggu KFS dipilih...")
            self.decision_box.configure(state="disabled")
            return

        # display_chase = [self.total_cells - 1 - i for i in self.robot_pos["KFS R2"]]
        # display_avoid = [self.total_cells - 1 - i for i in self.robot_pos["KFS R1"] + self.robot_pos["KFS FAKE"]]

        # #=====untuk tentukan mode nya====
        # if has_chase and has_avoid:
        #     mode = 3 #mixed
        # elif has_chase:
        #     mode = 1 #chase
        # elif has_avoid:
        #     mode = 2 #avoid
        # else:
        #     mode = 0 #idle

        decision = {
            "mode" : mode,
            "chase" : display_chase,
            "avoid" : display_avoid
        }

        self.decision_box.insert(END, "preview Decision:\n")
        self.decision_box.insert(END, str(decision))

        self.decision_box.configure(state='disabled')


    def run(self):
        self.root.mainloop()

def main():
    rclpy.init()
    gui_node = GridGUI()
    try:
        gui_node.root.mainloop()
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
    # gui = GridGUI()
    # gui.run()