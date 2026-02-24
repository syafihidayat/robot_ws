import ttkbootstrap as tb
from ttkbootstrap.constants import *
from tkinter import END,messagebox

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
GRID_ROWS = 4
GRID_COLS = 3

class GridGUI(Node):
    def __init__(self):

        super().__init__('grid_gui_node')
        
        # posisi robot
        self.robot_pos = {"KFS R1": [], "KFS R2": [], "KFS FAKE": []}
        self.active_robot = "KFS R1"

        self.chase_pub = self.create_publisher(Point, 'chase_from_backend',10)
        self.avoid_pub = self.create_publisher(Point, 'avoid_from_backend',10)

        # Window ttkbootstrap
        self.root = tb.Window(themename="flatly")
        self.root.title("Robot Grid Dashboard")
        self.root.geometry("750x650")

        self.buttons = []
        self.display_to_index = {}  # mapping display_number → index asli

        self.build_gui()
        self.update_decision()

        self.root.after(100,self.ros_spin)

    def ros_spin(self):
        rclpy.spin_once(self, timeout_sec=0)
        self.root.after(50, self.ros_spin)

    def build_gui(self):
        # Top frame untuk robot select + counter
        top = tb.Frame(self.root)
        top.pack(pady=15, fill=X)

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
            limits = {"KFS R1": 3, "KFS R2": 3, "KFS FAKE": 1}
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
        limits = {"KFS R1": 3, "KFS R2": 3, "KFS FAKE": 1}
        for r, counter in self.robot_counters.items():
            counter.configure(text=f"{len(self.robot_pos[r])}/{limits[r]}")

    def clear_all(self):
        for key in self.robot_pos:
            self.robot_pos[key] = []
        self.render()
        self.update_decision()

    def update_decision(self):
        self.decision_box.configure(state='normal')
        self.decision_box.delete('1.0',END)

        all_filled = any(len(v) > 0 for v in self.robot_pos.values())
        if not all_filled:
            self.decision_box.insert(END, "Menunggu semua KFS ditempatkan...")
        else:
            # tampilkan angka sesuai display_number
            display_chase = [self.total_cells - 1 - i for i in self.robot_pos["KFS R2"]]
            display_avoid = [self.total_cells - 1 - i for i in self.robot_pos["KFS R1"] + self.robot_pos["KFS FAKE"]]
            decision = {
                "mode": "CHASE_KFS R2",
                "chase": display_chase,
                "avoid": display_avoid
            }
            self.decision_box.insert(END, str(decision))

            print("publishing chase:", display_chase)
            for i in display_chase:
                msg = Point()
                msg.x = float(i)
                msg.y = 0.0
                msg.z = 0.0
                self.chase_pub.publish(msg)

            for i in display_avoid:
                msg = Point()
                msg.x = float(i)
                msg.y = 0.0
                msg.z = 0.0
                self.avoid_pub.publish(msg)

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