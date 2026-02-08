#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import threading
import subprocess
import tempfile
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

from PIL import Image, ImageTk, ImageFilter, ImageOps
from svgpathtools import svg2paths


class ImageToGcodeApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Image to G-code Converter (Improved Outline)")
        self.root.geometry("1100x700")

        # -------- Variables --------
        self.input_path = tk.StringVar()
        self.output_dir = tk.StringVar()
        self.output_filename = tk.StringVar(value="output")

        # Preprocess params
        self.invert = tk.BooleanVar(value=False)
        self.blur = tk.DoubleVar(value=0.6)
        self.threshold = tk.IntVar(value=160)   # Typical for black bg + white text: tune
        self.close_size = tk.IntVar(value=3)    # Morphological close kernel size: 3/5/7

        # Potrace params
        self.turdsize = tk.IntVar(value=10)     # Remove small speckles
        self.alphamax = tk.DoubleVar(value=1.0) # Corner smoothing
        self.opttol = tk.DoubleVar(value=0.2)   # Curve optimization tolerance

        # G-code params
        self.feed_rate = tk.IntVar(value=800)
        self.pen_up_z = tk.DoubleVar(value=1.0)
        self.pen_down_z = tk.DoubleVar(value=0.0)
        self.svg_scale = tk.DoubleVar(value=1.0)     # scale from SVG units to mm-ish (pre-normalize)
        self.target_size = tk.DoubleVar(value=25.0)  # final max dimension in mm

        # Preview
        self.original_image_L = None
        self.preview_imgs = {
            "original": None,
            "binary": None,
            "clean": None
        }
        self.preview_photo = {
            "original": None,
            "binary": None,
            "clean": None
        }
        self.preview_update_id = None

        self.create_widgets()

        # Trigger preview when params change
        for var in [self.threshold, self.blur, self.invert, self.close_size]:
            var.trace_add("write", self.schedule_preview_update)

    # ---------------- UI ----------------
    def create_widgets(self):
        main = ttk.Frame(self.root)
        main.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        left = ttk.Frame(main)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        right = ttk.Frame(main)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # -------- Controls --------
        r = 0
        ttk.Label(left, text="Input Image:").grid(row=r, column=0, sticky="w", pady=4)
        ttk.Entry(left, textvariable=self.input_path, width=40).grid(row=r, column=1, padx=5, pady=4)
        ttk.Button(left, text="Browse...", command=self.browse_input).grid(row=r, column=2, pady=4)
        r += 1

        ttk.Label(left, text="Output Directory:").grid(row=r, column=0, sticky="w", pady=4)
        ttk.Entry(left, textvariable=self.output_dir, width=40).grid(row=r, column=1, padx=5, pady=4)
        ttk.Button(left, text="Browse...", command=self.browse_output_dir).grid(row=r, column=2, pady=4)
        r += 1

        ttk.Label(left, text="Output Filename:").grid(row=r, column=0, sticky="w", pady=4)
        ttk.Entry(left, textvariable=self.output_filename, width=40).grid(row=r, column=1, columnspan=2, padx=5, pady=4, sticky="w")
        r += 1

        sep = ttk.Separator(left, orient="horizontal")
        sep.grid(row=r, column=0, columnspan=3, sticky="ew", pady=10)
        r += 1

        # Preprocess group
        pp = ttk.LabelFrame(left, text="Preprocess (Preview)")
        pp.grid(row=r, column=0, columnspan=3, sticky="ew", pady=6)
        r += 1

        rr = 0
        ttk.Checkbutton(pp, text="Invert", variable=self.invert).grid(row=rr, column=0, sticky="w", padx=6, pady=4)
        rr += 1

        ttk.Label(pp, text="Blur (0~3):").grid(row=rr, column=0, sticky="w", padx=6)
        ttk.Spinbox(pp, from_=0.0, to=3.0, increment=0.1, textvariable=self.blur, width=8).grid(row=rr, column=1, sticky="w", padx=6, pady=4)
        rr += 1

        ttk.Label(pp, text="Threshold (0~255):").grid(row=rr, column=0, sticky="w", padx=6)
        thr_frame = ttk.Frame(pp)
        thr_frame.grid(row=rr, column=1, sticky="ew", padx=6, pady=4)
        ttk.Scale(thr_frame, from_=0, to=255, variable=self.threshold, orient=tk.HORIZONTAL,
                  command=lambda x: self.threshold_label.config(text=str(self.threshold.get()))).pack(side=tk.TOP, fill=tk.X)
        self.threshold_label = ttk.Label(thr_frame, text=str(self.threshold.get()))
        self.threshold_label.pack(side=tk.TOP)
        rr += 1

        ttk.Label(pp, text="Close kernel size (odd):").grid(row=rr, column=0, sticky="w", padx=6)
        ttk.Spinbox(pp, from_=1, to=15, increment=2, textvariable=self.close_size, width=8).grid(row=rr, column=1, sticky="w", padx=6, pady=4)
        rr += 1

        # Potrace group
        pot = ttk.LabelFrame(left, text="Potrace (Noise & Smooth)")
        pot.grid(row=r, column=0, columnspan=3, sticky="ew", pady=6)
        r += 1

        rr = 0
        ttk.Label(pot, text="turdsize:").grid(row=rr, column=0, sticky="w", padx=6, pady=4)
        ttk.Spinbox(pot, from_=0, to=200, increment=1, textvariable=self.turdsize, width=10).grid(row=rr, column=1, sticky="w", padx=6, pady=4)
        rr += 1

        ttk.Label(pot, text="alphamax:").grid(row=rr, column=0, sticky="w", padx=6, pady=4)
        ttk.Spinbox(pot, from_=0.0, to=2.0, increment=0.1, textvariable=self.alphamax, width=10).grid(row=rr, column=1, sticky="w", padx=6, pady=4)
        rr += 1

        ttk.Label(pot, text="opttolerance:").grid(row=rr, column=0, sticky="w", padx=6, pady=4)
        ttk.Spinbox(pot, from_=0.0, to=1.0, increment=0.05, textvariable=self.opttol, width=10).grid(row=rr, column=1, sticky="w", padx=6, pady=4)
        rr += 1

        # G-code group
        gc = ttk.LabelFrame(left, text="G-code")
        gc.grid(row=r, column=0, columnspan=3, sticky="ew", pady=6)
        r += 1

        rr = 0
        ttk.Label(gc, text="Feed rate:").grid(row=rr, column=0, sticky="w", padx=6, pady=4)
        ttk.Spinbox(gc, from_=100, to=8000, increment=100, textvariable=self.feed_rate, width=10).grid(row=rr, column=1, sticky="w", padx=6, pady=4)
        rr += 1

        ttk.Label(gc, text="Pen up Z:").grid(row=rr, column=0, sticky="w", padx=6, pady=4)
        ttk.Spinbox(gc, from_=0.0, to=50.0, increment=0.1, textvariable=self.pen_up_z, width=10).grid(row=rr, column=1, sticky="w", padx=6, pady=4)
        rr += 1

        ttk.Label(gc, text="Pen down Z:").grid(row=rr, column=0, sticky="w", padx=6, pady=4)
        ttk.Spinbox(gc, from_=-10.0, to=10.0, increment=0.1, textvariable=self.pen_down_z, width=10).grid(row=rr, column=1, sticky="w", padx=6, pady=4)
        rr += 1

        ttk.Label(gc, text="SVG scale (pre):").grid(row=rr, column=0, sticky="w", padx=6, pady=4)
        ttk.Spinbox(gc, from_=0.01, to=20.0, increment=0.1, textvariable=self.svg_scale, width=10).grid(row=rr, column=1, sticky="w", padx=6, pady=4)
        rr += 1

        ttk.Label(gc, text="Target size (mm):").grid(row=rr, column=0, sticky="w", padx=6, pady=4)
        ttk.Spinbox(gc, from_=1.0, to=500.0, increment=1.0, textvariable=self.target_size, width=10).grid(row=rr, column=1, sticky="w", padx=6, pady=4)
        rr += 1

        ttk.Button(left, text="Convert to G-code", command=self.convert).grid(row=r, column=0, columnspan=3, sticky="ew", pady=12)
        r += 1

        # -------- Preview tabs --------
        nb = ttk.Notebook(right)
        nb.pack(fill=tk.BOTH, expand=True)

        self.tab_original = ttk.Frame(nb)
        self.tab_binary = ttk.Frame(nb)
        self.tab_clean = ttk.Frame(nb)

        nb.add(self.tab_original, text="Original")
        nb.add(self.tab_binary, text="Binary")
        nb.add(self.tab_clean, text="Cleaned")

        self.lbl_original = ttk.Label(self.tab_original, text="No image", background="white")
        self.lbl_binary = ttk.Label(self.tab_binary, text="No image", background="white")
        self.lbl_clean = ttk.Label(self.tab_clean, text="No image", background="white")

        for lbl in [self.lbl_original, self.lbl_binary, self.lbl_clean]:
            lbl.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Status bar
        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)\
            .pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)

    # ---------------- File dialogs ----------------
    def browse_input(self):
        filetypes = (('Image files', '*.jpg *.jpeg *.png *.bmp *.gif'), ('All files', '*.*'))
        fn = filedialog.askopenfilename(title="Select an image file", filetypes=filetypes)
        if not fn:
            return
        self.input_path.set(fn)
        if not self.output_filename.get() or self.output_filename.get() == "output":
            self.output_filename.set(os.path.splitext(os.path.basename(fn))[0])

        try:
            self.original_image_L = Image.open(fn).convert("L")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load image: {e}")
            return

        self.update_preview()

    def browse_output_dir(self):
        d = filedialog.askdirectory(title="Select output directory")
        if d:
            self.output_dir.set(d)

    # ---------------- Preview pipeline ----------------
    def schedule_preview_update(self, *args):
        if self.preview_update_id:
            self.root.after_cancel(self.preview_update_id)
        self.preview_update_id = self.root.after(250, self.update_preview)

    def update_preview(self):
        if self.original_image_L is None:
            return
        t = threading.Thread(target=self._preview_worker, daemon=True)
        t.start()

    def _preview_worker(self):
        try:
            img_L = self.original_image_L.copy()

            # Original preview
            self.preview_imgs["original"] = img_L

            # Preprocess => binary + clean
            binary_1bit, clean_1bit = self.preprocess_to_binary(img_L)

            self.preview_imgs["binary"] = binary_1bit.convert("L")
            self.preview_imgs["clean"] = clean_1bit.convert("L")

            self.root.after(0, self._render_previews)

        except Exception as e:
            self.root.after(0, lambda: self.status_var.set(f"Preview error: {e}"))

    def _render_previews(self):
        self._set_label_image(self.lbl_original, self.preview_imgs["original"], "original")
        self._set_label_image(self.lbl_binary, self.preview_imgs["binary"], "binary")
        self._set_label_image(self.lbl_clean, self.preview_imgs["clean"], "clean")

    def _set_label_image(self, label, pil_img, key):
        if pil_img is None:
            return
        img = pil_img.convert("RGB")
        img.thumbnail((700, 650), Image.Resampling.LANCZOS)
        photo = ImageTk.PhotoImage(img)
        self.preview_photo[key] = photo
        label.config(image=photo, text="")

    # ---------------- Core processing ----------------
    def preprocess_to_binary(self, img_L: Image.Image):
        """
        Produce a clean 1-bit image where foreground strokes are BLACK (0) and background is WHITE (255).
        """
        # Optional blur
        blur = float(self.blur.get())
        if blur > 0:
            img_L = img_L.filter(ImageFilter.GaussianBlur(radius=blur))

        # Optional invert
        if self.invert.get():
            img_L = ImageOps.invert(img_L)

        thr = int(self.threshold.get())

        # Binary: foreground black (0) for potrace; background white (255)
        # Rule: pixels darker than threshold => black (stroke), else white
        binary = img_L.point(lambda x: 0 if x < thr else 255, mode="L").convert("1")

        # Morphological close (dilate then erode) to fill small gaps
        k = int(self.close_size.get())
        if k < 1:
            k = 1
        if k % 2 == 0:
            k += 1

        if k > 1:
            dilated = binary.filter(ImageFilter.MaxFilter(size=k))
            closed = dilated.filter(ImageFilter.MinFilter(size=k))
        else:
            closed = binary

        return binary, closed

    def bitmap_to_svg_with_potrace(self, binary_1bit: Image.Image, svg_path: str):
        """
        Save binary as PBM and call potrace to produce SVG.
        """
        with tempfile.TemporaryDirectory() as td:
            pbm_path = os.path.join(td, "input.pbm")
            binary_1bit.save(pbm_path)

            cmd = [
                "potrace", pbm_path,
                "-s", "-o", svg_path,
                "--turdsize", str(int(self.turdsize.get())),
                "--alphamax", str(float(self.alphamax.get())),
                "--opttolerance", str(float(self.opttol.get())),
            ]

            # Run potrace
            subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def svg_to_gcode_polyline(self, svg_file: str, gcode_file: str):
        """
        Convert SVG paths directly to G-code polylines (no shapely union).
        """
        paths, _ = svg2paths(svg_file)

        pen_up = float(self.pen_up_z.get())
        pen_down = float(self.pen_down_z.get())
        feed = int(self.feed_rate.get())
        scale = float(self.svg_scale.get())

        # Sample step in SVG units (smaller => smoother but longer G-code)
        # You can tune this.
        base_step = 2.0

        def sample_path_to_points(path):
            try:
                L = float(path.length(error=1e-3))
            except Exception:
                L = 200.0
            n = max(30, min(2000, int(L / base_step) + 1))
            pts = []
            for i in range(n + 1):
                t = i / n
                p = path.point(t)
                x = p.real * scale
                y = (-p.imag) * scale  # Flip Y: SVG down -> CNC up
                pts.append((x, y))
            # Remove consecutive duplicates
            compact = []
            last = None
            for xy in pts:
                if last is None or (abs(xy[0] - last[0]) > 1e-6 or abs(xy[1] - last[1]) > 1e-6):
                    compact.append(xy)
                    last = xy
            return compact

        # Write raw gcode
        with open(gcode_file, "w", encoding="utf-8") as f:
            f.write("; Outline G-code (improved pipeline)\n")
            f.write("G21 ; mm units\n")
            f.write("G90 ; absolute positioning\n")
            f.write(f"G1 F{feed}\n")

            any_path = False
            for path in paths:
                pts = sample_path_to_points(path)
                if len(pts) < 2:
                    continue
                any_path = True

                x0, y0 = pts[0]
                f.write(f"G0 Z{pen_up:.2f}\n")
                f.write(f"G0 X{x0:.2f} Y{y0:.2f}\n")
                f.write(f"G1 Z{pen_down:.2f}\n")
                for (x, y) in pts[1:]:
                    f.write(f"G1 X{x:.2f} Y{y:.2f}\n")
                f.write(f"G0 Z{pen_up:.2f}\n")

            f.write("M2 ; end\n")

        if not any_path:
            raise ValueError("No valid paths found in SVG (try lowering threshold or reducing turdsize).")

    def normalize_gcode_to_target(self, gcode_path: str, target_size_mm: float):
        """
        Shift to (0,0) and scale so that max(width, height) = target_size_mm.
        """
        with open(gcode_path, "r", encoding="utf-8") as f:
            lines = f.readlines()

        x_vals, y_vals = [], []
        for line in lines:
            for axis, val in re.findall(r'([XY])([-+]?\d*\.?\d+)', line):
                if axis == "X":
                    x_vals.append(float(val))
                elif axis == "Y":
                    y_vals.append(float(val))

        if not x_vals or not y_vals:
            raise ValueError("No X/Y found in gcode, normalization aborted.")

        xmin, xmax = min(x_vals), max(x_vals)
        ymin, ymax = min(y_vals), max(y_vals)
        w = xmax - xmin
        h = ymax - ymin
        max_dim = max(w, h)
        if max_dim <= 1e-9:
            raise ValueError("Degenerate bounding box, normalization aborted.")

        s = float(target_size_mm) / max_dim

        def repl(m):
            axis = m.group(1)
            v = float(m.group(2))
            if axis == "X":
                return f"X{(v - xmin) * s:.2f}"
            if axis == "Y":
                return f"Y{(v - ymin) * s:.2f}"
            return m.group(0)

        out = []
        for line in lines:
            out.append(re.sub(r'([XY])([-+]?\d*\.?\d+)', repl, line))

        with open(gcode_path, "w", encoding="utf-8") as f:
            f.writelines(out)

        return (xmin, xmax, ymin, ymax, s)

    # ---------------- Convert pipeline ----------------
    def convert(self):
        input_path = self.input_path.get()
        out_dir = self.output_dir.get()
        out_name = self.output_filename.get().strip()

        if not input_path or not os.path.isfile(input_path):
            messagebox.showerror("Error", "Please select a valid input image.")
            return
        if not out_dir or not os.path.isdir(out_dir):
            messagebox.showerror("Error", "Please select a valid output directory.")
            return
        if not out_name:
            messagebox.showerror("Error", "Please enter an output filename.")
            return

        svg_path = os.path.join(out_dir, f"{out_name}.svg")
        gcode_path = os.path.join(out_dir, f"{out_name}.gcode")

        t = threading.Thread(
            target=self._convert_worker,
            args=(input_path, svg_path, gcode_path),
            daemon=True
        )
        t.start()

    def _convert_worker(self, input_path: str, svg_path: str, gcode_path: str):
        try:
            self.root.after(0, lambda: self.status_var.set("Loading image..."))
            img_L = Image.open(input_path).convert("L")

            self.root.after(0, lambda: self.status_var.set("Preprocessing to clean binary..."))
            _, clean_1bit = self.preprocess_to_binary(img_L)

            self.root.after(0, lambda: self.status_var.set("Running potrace -> SVG..."))
            self.bitmap_to_svg_with_potrace(clean_1bit, svg_path)

            self.root.after(0, lambda: self.status_var.set("SVG -> G-code (polyline sampling)..."))
            self.svg_to_gcode_polyline(svg_path, gcode_path)

            self.root.after(0, lambda: self.status_var.set("Normalizing G-code to target size..."))
            bbox = self.normalize_gcode_to_target(gcode_path, float(self.target_size.get()))

            xmin, xmax, ymin, ymax, s = bbox
            msg = (
                f"Done!\n\n"
                f"SVG:  {svg_path}\n"
                f"GCODE:{gcode_path}\n\n"
                f"Raw bbox: X[{xmin:.2f},{xmax:.2f}] Y[{ymin:.2f},{ymax:.2f}]\n"
                f"Final scale factor applied: {s:.6f}\n"
            )

            self.root.after(0, lambda: self.status_var.set("Conversion completed successfully!"))
            self.root.after(0, lambda: messagebox.showinfo("Success", msg))

        except subprocess.CalledProcessError as e:
            err = ""
            try:
                err = (e.stderr or b"").decode("utf-8", errors="ignore")
            except Exception:
                err = str(e)
            self.root.after(0, lambda: self.status_var.set("Failed"))
            self.root.after(0, lambda: messagebox.showerror(
                "Potrace Error",
                f"potrace failed.\n\nMake sure potrace is installed and in PATH.\n\nDetails:\n{err}"
            ))

        except Exception as e:
            self.root.after(0, lambda: self.status_var.set("Failed"))
            self.root.after(0, lambda: messagebox.showerror("Error", str(e)))


if __name__ == "__main__":
    root = tk.Tk()
    app = ImageToGcodeApp(root)
    root.mainloop()
