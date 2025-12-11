"""Raw Data tab UI construction."""

import tkinter as tk
from tkinter import ttk


class RawDataTabMixin:
    def build_raw_data_sheet(self, parent: ttk.Frame):
        """Build a spreadsheet-like grid for raw data."""
        self.sheet_cells = {}

        # Create Scrollable Canvas
        canvas = tk.Canvas(parent, bg=self.palette["bg"], highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas, style="App.TFrame")

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Mousewheel scrolling
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        canvas.bind_all("<MouseWheel>", _on_mousewheel)
        # Linux Support
        canvas.bind_all("<Button-4>", lambda e: canvas.yview_scroll(-1, "units"))
        canvas.bind_all("<Button-5>", lambda e: canvas.yview_scroll(1, "units"))

        canvas.grid(column=0, row=0, sticky="nsew")
        scrollbar.grid(column=1, row=0, sticky="ns")
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=1)

        # --- Spreadsheet Construction ---

        # Helper for creating cells
        def make_header(p, text, r, c, w=20):
            lbl = ttk.Label(p, text=text, style="SheetHeader.TLabel", width=w, anchor="center")
            lbl.grid(row=r, column=c, sticky="nsew", padx=1, pady=1)
            return lbl

        def make_label(p, text, r, c, w=20):
            lbl = ttk.Label(p, text=text, style="SheetLabel.TLabel", width=w, anchor="e")
            lbl.grid(row=r, column=c, sticky="nsew", padx=1, pady=1)
            return lbl

        def make_value(p, key, r, c, w=40):
            lbl = ttk.Label(p, text="0", style="SheetCell.TLabel", width=w, anchor="w")
            lbl.grid(row=r, column=c, sticky="nsew", padx=1, pady=1)
            self.sheet_cells[key] = lbl
            return lbl

        # SECTION 1: Headset & System (Row 0)
        sec1 = ttk.LabelFrame(scrollable_frame, text="Headset System", style="Section.TLabelframe")
        sec1.grid(row=0, column=0, columnspan=2, sticky="ew", padx=8, pady=8)

        make_header(sec1, "Property", 0, 0)
        make_header(sec1, "Value", 0, 1, w=60)

        make_label(sec1, "Pose", 1, 0)
        make_value(sec1, "headset_pose", 1, 1)

        make_label(sec1, "Status", 2, 0)
        make_value(sec1, "headset_status", 2, 1)

        make_label(sec1, "Timestamp", 3, 0)
        make_value(sec1, "headset_time", 3, 1)

        # SECTION 2: Controllers (Row 1)
        sec2 = ttk.LabelFrame(scrollable_frame, text="Controllers", style="Section.TLabelframe")
        sec2.grid(row=1, column=0, columnspan=2, sticky="ew", padx=8, pady=4)

        make_header(sec2, "Property", 0, 0)
        make_header(sec2, "Left Controller", 0, 1, w=45)
        make_header(sec2, "Right Controller", 0, 2, w=45)

        rows = ["Pose", "Axis (X,Y)", "Trigger / Grip", "Buttons"]
        keys = ["c_pose", "c_axis", "c_trigger", "c_buttons"]

        for i, (label, key_suffix) in enumerate(zip(rows, keys)):
            r = i + 1
            make_label(sec2, label, r, 0)
            make_value(sec2, f"left_{key_suffix}", r, 1, w=45)
            make_value(sec2, f"right_{key_suffix}", r, 2, w=45)

        # SECTION 3: Hands Summary (Row 2)
        sec3 = ttk.LabelFrame(scrollable_frame, text="Hands Summary", style="Section.TLabelframe")
        sec3.grid(row=2, column=0, columnspan=2, sticky="ew", padx=8, pady=4)

        make_header(sec3, "Property", 0, 0)
        make_header(sec3, "Left Hand", 0, 1, w=45)
        make_header(sec3, "Right Hand", 0, 2, w=45)

        make_label(sec3, "Active", 1, 0)
        make_value(sec3, "left_h_active", 1, 1, w=45)
        make_value(sec3, "right_h_active", 1, 2, w=45)

        make_label(sec3, "Scale", 2, 0)
        make_value(sec3, "left_h_scale", 2, 1, w=45)
        make_value(sec3, "right_h_scale", 2, 2, w=45)

        # SECTION 4: Hand Joints (Row 3) - The Big Table
        sec4 = ttk.LabelFrame(scrollable_frame, text="Hand Joints (26)", style="Section.TLabelframe")
        sec4.grid(row=3, column=0, columnspan=2, sticky="ew", padx=8, pady=4)

        make_header(sec4, "Joint #", 0, 0)
        make_header(sec4, "Left Hand", 0, 1, w=60)
        make_header(sec4, "Right Hand", 0, 2, w=60)

        for i in range(26):
            make_label(sec4, f"{i}", i + 1, 0)
            make_value(sec4, f"joint_{i}_left", i + 1, 1, w=60)
            make_value(sec4, f"joint_{i}_right", i + 1, 2, w=60)

