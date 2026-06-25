"""
app_main.py — Wrapper 2 halaman (bisa di-slide) untuk Meihua Forest Planner.

  Halaman 1  = GUI mehua.py YANG SUDAH ADA — TIDAK DIUBAH SAMA SEKALI.
  Halaman 2  = kosong dulu (placeholder), nanti diisi belakangan.

File mehua.py HARUS ada di folder yang sama dengan file ini, dan
isinya 100% sama dengan yang sudah kamu pakai sekarang — tidak ada
satu baris pun yang disentuh.

Cara jalankan (sama seperti mehua.py aslinya):
    python app_main.py            (dengan ROS2)
    python app_main.py --no-ros   (tanpa ROS2, GUI only)

Navigasi antar halaman:
  • Klik tombol "‹" / "›" di pojok bawah, ATAU
  • klik titik indikator di tengah bawah, ATAU
  • klik-drag (swipe) pada area kosong / halaman 2.
    (Swipe dari atas tombol/canvas di halaman 1 tidak akan jalan,
     karena tombol & canvas itu sendiri yang "menangkap" klik —
     ini wajar karena halaman 1 memang tidak diubah/ditambah apa pun.)
"""

import tkinter as tk

# Import APA ADANYA dari mehua.py — tidak ada satu baris di mehua.py yang diubah.
from .mehua import MeihuaApp, BG, PANEL_BG, BORDER, ACCENT

class _RootProxy(tk.Frame):
    """
    Frame yang "menyamar" jadi window asli untuk MeihuaApp.

    mehua.py ditulis dengan asumsi dia menerima objek root Tk() asli, jadi di
    __init__-nya dia langsung panggil:
        self.root.title(...)
        self.root.resizable(...)
        self.root.protocol("WM_DELETE_WINDOW", ...)

    Frame biasa tidak punya method-method itu. Supaya mehua.py TIDAK PERLU
    DIUBAH SAMA SEKALI tapi tetap bisa ditaruh sebagai satu halaman (bukan
    jadi window-nya sendiri), proxy ini menangkap panggilan tersebut dan
    meneruskannya ke window asli (real_root).
    """

    def __init__(self, master, real_root, **kwargs):
        super().__init__(master, **kwargs)
        self._real_root = real_root

    def title(self, *args, **kwargs):
        if args:
            self._real_root.title(*args, **kwargs)
        return self._real_root.title()

    def resizable(self, *args, **kwargs):
        return self._real_root.resizable(*args, **kwargs)

    def protocol(self, *args, **kwargs):
        return self._real_root.protocol(*args, **kwargs)

    def destroy(self):
        # Saat mehua.py minta tutup window (mis. lewat _on_close bawaannya),
        # tutup window ASLI juga — bukan cuma frame halaman ini.
        try:
            self._real_root.destroy()
        except Exception:
            pass


class SlidingApp:
    FOOTER_H = 34

    def __init__(self, root):
        self.root = root
        self.root.configure(bg=BG)

        self.offset = 0
        self.current_page = 0
        self._animating = False
        self._drag_start_x = None
        self._drag_start_offset = 0

        self._build()

    # ---------------------------------------------------------
    def _build(self):
        self.container = tk.Frame(self.root, bg=BG)

        # ── Halaman 1 : GUI mehua.py, TIDAK DIUBAH ─────────────
        self.page1 = _RootProxy(self.container, real_root=self.root, bg=BG)
        self.meihua_app = MeihuaApp(self.page1)   # <- GUI asli, apa adanya

        # Ukur kebutuhan ukuran halaman 1 setelah semua widget-nya jadi,
        # supaya jendela tetap pas seperti aslinya (tidak terpotong).
        self.root.update_idletasks()
        self.PAGE_W = self.page1.winfo_reqwidth()
        self.PAGE_H = self.page1.winfo_reqheight()

        # ── Halaman 2 : kosong dulu ─────────────────────────────
        self.page2 = tk.Frame(self.container, bg=BG,
                               width=self.PAGE_W, height=self.PAGE_H)
        tk.Label(self.page2, text="Halaman 2 — (kosong, untuk nanti)",
                 bg=BG, fg="#64748b", font=("Courier", 10)).place(
                     relx=0.5, rely=0.5, anchor="center")

        self.page1.place(x=0, y=0, width=self.PAGE_W, height=self.PAGE_H)
        self.page2.place(x=self.PAGE_W, y=0, width=self.PAGE_W, height=self.PAGE_H)

        self.container.configure(width=self.PAGE_W, height=self.PAGE_H)
        self.container.pack_propagate(False)

        # ── Footer navigasi (panah + titik), selalu tampil di bawah ─
        self.footer = tk.Frame(self.root, bg=PANEL_BG, height=self.FOOTER_H,
                                highlightbackground=BORDER, highlightthickness=1)
        self.footer.pack_propagate(False)
        self._build_footer()

        # footer dulu (menempel di bawah), baru container isi sisa ruang
        self.footer.pack(side="bottom", fill="x")
        self.container.pack(side="top", fill="both", expand=True)

        # swipe aktif di container & halaman 2 (area kosong).
        for w in (self.container, self.page2):
            w.bind("<ButtonPress-1>", self._on_drag_start)
            w.bind("<B1-Motion>", self._on_drag_motion)
            w.bind("<ButtonRelease-1>", self._on_drag_end)

    def _build_footer(self):
        tk.Button(
            self.footer, text="‹", bg=PANEL_BG, fg=ACCENT, bd=0,
            font=("Courier", 12, "bold"),
            activebackground=PANEL_BG, activeforeground=ACCENT,
            command=lambda: self.go_to_page(0)
        ).pack(side="left", padx=10)

        dots_frame = tk.Frame(self.footer, bg=PANEL_BG)
        dots_frame.pack(side="left", expand=True)
        self.dot_labels = []
        for i in range(2):
            lbl = tk.Label(dots_frame, text="●", bg=PANEL_BG,
                            fg=(ACCENT if i == 0 else "#475569"),
                            font=("Courier", 10))
            lbl.pack(side="left", padx=4)
            self.dot_labels.append(lbl)

        tk.Button(
            self.footer, text="›", bg=PANEL_BG, fg=ACCENT, bd=0,
            font=("Courier", 12, "bold"),
            activebackground=PANEL_BG, activeforeground=ACCENT,
            command=lambda: self.go_to_page(1)
        ).pack(side="right", padx=10)

    def _update_dots(self):
        for i, lbl in enumerate(self.dot_labels):
            lbl.configure(fg=(ACCENT if i == self.current_page else "#475569"))

    # ---------------------------------------------------------
    # Slide / animasi
    # ---------------------------------------------------------
    def _set_offset(self, x):
        self.offset = x
        self.page1.place_configure(x=x)
        self.page2.place_configure(x=x + self.PAGE_W)

    def go_to_page(self, page_index, animate=True):
        target = -page_index * self.PAGE_W
        self.current_page = page_index
        self._update_dots()
        if animate:
            self._animate_to(target)
        else:
            self._set_offset(target)

    def _animate_to(self, target_x, steps=12, delay=12):
        if self._animating:
            return
        self._animating = True
        start_x = self.offset
        distance = target_x - start_x
        step_count = [0]

        def step():
            step_count[0] += 1
            t = step_count[0] / steps
            eased = 1 - (1 - t) ** 2  # ease-out
            self._set_offset(int(start_x + distance * eased))
            if step_count[0] < steps:
                self.root.after(delay, step)
            else:
                self._set_offset(target_x)
                self._animating = False

        step()

    def _on_drag_start(self, event):
        if self._animating:
            return
        self._drag_start_x = event.x_root
        self._drag_start_offset = self.offset

    def _on_drag_motion(self, event):
        if self._drag_start_x is None or self._animating:
            return
        dx = event.x_root - self._drag_start_x
        new_offset = self._drag_start_offset + dx
        new_offset = max(-self.PAGE_W, min(0, new_offset))
        self._set_offset(new_offset)

    def _on_drag_end(self, event):
        if self._drag_start_x is None:
            return
        self._drag_start_x = None
        if self.offset <= -self.PAGE_W / 2:
            self.go_to_page(1)
        else:
            self.go_to_page(0)


def main():
    root = tk.Tk()
    SlidingApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()