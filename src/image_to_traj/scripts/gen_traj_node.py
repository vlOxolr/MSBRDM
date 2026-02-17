#!/usr/bin/env python3
import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patheffects as patheffects
import xml.etree.ElementTree as ET

from skimage.io import imread
from skimage.color import rgb2gray

# the same catalogue as trajectory_generator.py
from trajectory_generator import TrajectoryGenerator

import rospy
from std_msgs.msg import String


def load_gray(image_path: str) -> np.ndarray:
    img = imread(image_path)
    if img.ndim == 3:
        gray = rgb2gray(img)  # [0,1]
    else:
        gray = img.astype(np.float32)
        if gray.max() > 1.0:
            gray /= 255.0
    return gray.astype(np.float32)


def parse_xml_trajectory(xml_str: str):
    """
    Returns: list of dict:
      [
        {"index": int, "coef_x": np.ndarray, "coef_y": np.ndarray, "length_px": float, "deg_used": int},
        ...
      ]
    """
    root = ET.fromstring(xml_str)
    strokes_el = root.find("strokes")
    if strokes_el is None:
        return []

    out = []
    for st in strokes_el.findall("stroke"):
        idx = int(st.get("index", "0"))
        deg_used = int(st.get("deg_used", "0"))

        length_el = st.find("length_px")
        length_px = float(length_el.text) if (length_el is not None and length_el.text) else 0.0

        fit_el = st.find("polyfit")
        if fit_el is None:
            continue

        x_el = fit_el.find("x")
        y_el = fit_el.find("y")
        if x_el is None or y_el is None or (x_el.text is None) or (y_el.text is None):
            continue

        # coefficients are comma-separated, highest power -> lowest power
        coef_x = np.array([float(v) for v in x_el.text.split(",") if v.strip() != ""], dtype=np.float64)
        coef_y = np.array([float(v) for v in y_el.text.split(",") if v.strip() != ""], dtype=np.float64)

        out.append({
            "index": idx,
            "deg_used": deg_used,
            "coef_x": coef_x,
            "coef_y": coef_y,
            "length_px": length_px,
        })

    out.sort(key=lambda d: d["index"])
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=str, required=True, help="Image path")
    parser.add_argument("--xml_out", type=str, default=None, help="Optional: save XML to file")
    parser.add_argument("--show", action="store_true", help="Show visualization window")
    parser.add_argument("--save_fig", type=str, default=None, help="Optional: save visualization png path")
    parser.add_argument("--dense", type=int, default=800, help="Dense samples for drawing fitted curve")
    parser.add_argument("--print_lengths", action="store_true", help="Print each stroke length from XML")
    args = parser.parse_args()

    gen = TrajectoryGenerator()

    # 1) only run once to generate the XML
    xml_str = gen.generate_xml(args.image)
    print(xml_str)

    # ROS init
    rospy.init_node("trajectory_generator_node", anonymous=False)

    # Publish XML as a latched topic so late subscribers still receive it
    pub = rospy.Publisher("/ur10/planar_polynomial_trajectories", String, queue_size=1, latch=True)

    msg = String()
    msg.data = xml_str
    pub.publish(msg)
    rospy.loginfo(f"Published strokes XML to /ur10/planar_polynomial_trajectories (bytes={len(xml_str.encode('utf-8'))})")


    if args.xml_out:
        os.makedirs(os.path.dirname(args.xml_out) or ".", exist_ok=True)
        with open(args.xml_out, "w", encoding="utf-8") as f:
            f.write(xml_str)
        print(f"[INFO] XML saved to: {args.xml_out}")

    # 2) analyze XML to get fitting parameters + length
    strokes = parse_xml_trajectory(xml_str)
    if len(strokes) == 0:
        print("[WARN] No strokes parsed from XML. Abort visualization.")
        return

    if args.print_lengths:
        for s in strokes:
            print(f"[LEN] Stroke {s['index']}: length_px={s['length_px']:.6f} (deg_used={s['deg_used']})")

    # 3) background use the orginal image（grey image）
    gray = load_gray(args.image)

    # 4) visualization: use the coef_x/coef_y in XML to draw fitting lines
    t_dense = np.linspace(0.0, 1.0, max(200, int(args.dense)), endpoint=True)

    fig = plt.figure(figsize=(12, 8))
    ax = plt.gca()

    ax.imshow(gray, cmap=plt.cm.gray)
    ax.axis("off")
    ax.set_title("Fitted trajectories from XML (ordered strokes) on original image", fontsize=12)

    # White sequential text + black outline: clearer
    pe = patheffects.withStroke(linewidth=3, foreground="black")

    for s in strokes:
        i = s["index"]
        coef_x = s["coef_x"]  # col
        coef_y = s["coef_y"]  # row

        x_dense = np.polyval(coef_x, t_dense)
        y_dense = np.polyval(coef_y, t_dense)

        ax.plot(x_dense, y_dense, linewidth=2.0)

        x0 = float(x_dense[0])
        y0 = float(y_dense[0])

        ax.scatter([x0], [y0], s=35)
        ax.text(
            x0, y0, str(i),
            color="white",
            fontsize=12,
            fontweight="bold",
            ha="center", va="center",
            path_effects=[pe],
        )

    plt.tight_layout()

    if args.save_fig:
        os.makedirs(os.path.dirname(args.save_fig) or ".", exist_ok=True)
        plt.savefig(args.save_fig, dpi=200)
        print(f"[INFO] Figure saved to: {args.save_fig}")

    if args.show or (not args.save_fig):
        plt.show()

    # optional: keep node alive
    rospy.spin()


if __name__ == "__main__":
    main()
