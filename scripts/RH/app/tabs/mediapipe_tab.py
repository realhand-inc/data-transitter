"""MediaPipe tab UI and update helpers."""

from tkinter import ttk


class MediaPipeTabMixin:
    def build_mediapipe_tab(self, parent: ttk.Frame):
        """Display MediaPipe-converted hand keypoints for both hands."""
        self.mediapipe_tables = {}

        def build_table(col: int, title: str, key: str):
            frame = ttk.LabelFrame(parent, text=title, style="Section.TLabelframe")
            frame.grid(row=0, column=col, padx=8, pady=8, sticky="nsew")
            frame.columnconfigure(0, weight=1)
            frame.rowconfigure(0, weight=1)

            columns = ("joint", "x", "y", "z")
            tree = ttk.Treeview(frame, columns=columns, show="headings", height=22)
            tree.heading("joint", text="Joint #")
            tree.heading("x", text="X (rel)")
            tree.heading("y", text="Y (rel)")
            tree.heading("z", text="Z (rel)")
            tree.column("joint", width=60, anchor="center")
            tree.column("x", width=80, anchor="e")
            tree.column("y", width=80, anchor="e")
            tree.column("z", width=80, anchor="e")

            vsb = ttk.Scrollbar(frame, orient="vertical", command=tree.yview)
            tree.configure(yscrollcommand=vsb.set)
            tree.grid(row=0, column=0, sticky="nsew")
            vsb.grid(row=0, column=1, sticky="ns")

            # Pre-populate rows
            for idx in range(21):
                tree.insert("", "end", iid=str(idx), values=(idx, "--", "--", "--"))

            self.mediapipe_tables[key] = tree

            note = ttk.Label(frame, text="Relative to wrist (MediaPipe coords)", style="TLabel")
            note.grid(row=1, column=0, columnspan=2, sticky="w", padx=4, pady=(4, 2))

        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)
        parent.rowconfigure(0, weight=1)

        build_table(0, "Left Hand (MediaPipe)", "left")
        build_table(1, "Right Hand (MediaPipe)", "right")

    def update_mediapipe_table(self, side: str, mp_data):
        """Populate MediaPipe tables with converted hand data."""
        tree = self.mediapipe_tables.get(side)
        if not tree:
            return

        def set_row(idx, x_val, y_val, z_val):
            tree.set(str(idx), column="x", value=x_val)
            tree.set(str(idx), column="y", value=y_val)
            tree.set(str(idx), column="z", value=z_val)

        if mp_data is None or len(mp_data) == 0:
            for i in range(21):
                set_row(i, "--", "--", "--")
            return

        for i in range(21):
            if i < len(mp_data):
                x, y, z = mp_data[i]
                set_row(i, f"{x:.3f}", f"{y:.3f}", f"{z:.3f}")
            else:
                set_row(i, "--", "--", "--")
