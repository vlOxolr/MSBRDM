#!/usr/bin/env python3
import os
import glob
import argparse
from typing import Optional

import numpy as np
import matplotlib.pyplot as plt

from skimage.io import imread, imsave
from skimage.color import rgb2gray
from skimage.filters import threshold_otsu
from skimage.morphology import (
    white_tophat,
    opening,
    closing,
    skeletonize,
    rectangle,
    disk,
    medial_axis,
)


def resolve_img_path(script_dir: str, filename: Optional[str]) -> str:
    """Resolve an image path under scripts/img."""
    img_dir = os.path.join(script_dir, "img")
    if not os.path.isdir(img_dir):
        raise FileNotFoundError("Image folder not found: {}".format(img_dir))

    if filename:
        cand = os.path.join(img_dir, filename)
        if not os.path.isfile(cand):
            raise FileNotFoundError("Image not found: {}".format(cand))
        return cand

    exts = ("*.png", "*.jpg", "*.jpeg", "*.bmp", "*.tif", "*.tiff")
    files = []
    for e in exts:
        files.extend(glob.glob(os.path.join(img_dir, e)))
    files = sorted(files)

    if not files:
        raise FileNotFoundError("No images found in: {}".format(img_dir))
    return files[0]


def load_gray(image_path: str) -> np.ndarray:
    """Load image and convert to grayscale in [0, 1]."""
    img = imread(image_path)
    if img.ndim == 3:
        gray = rgb2gray(img)  # [0, 1]
    else:
        gray = img.astype(np.float32)
        if gray.max() > 1.0:
            gray /= 255.0
    return gray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--file",
        type=str,
        default=None,
        help="Image filename under scripts/img (e.g., test.png). If omitted, the first image is used.",
    )
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    image_path = resolve_img_path(script_dir, args.file)
    print("[INFO] Loading: {}".format(image_path))

    gray = load_gray(image_path)

    # --- Step 1: Binarization ---
    t1 = threshold_otsu(gray)
    binary1 = gray < t1

    # --- Step 2: White top-hat on inverted grayscale ---
    inv = 1.0 - gray
    TH_H = 15
    TH_W = 10
    selem_tophat = rectangle(TH_H, TH_W)
    tophat = white_tophat(inv, selem_tophat)

    # --- Step 3: Binarization after top-hat ---
    TH_T = 0.2
    binary2 = tophat > TH_T

    # --- Step 4: Opening ---
    OPEN_R = 1
    opened = opening(binary2, disk(OPEN_R))

    # --- Step 5: Closing (+ an extra opening as you added) ---
    CLOSE_R = 2
    cleaned = closing(opened, disk(CLOSE_R))
    cleaned = opening(cleaned, disk(OPEN_R))

    # --- Step 6a: Thinning skeleton (your original) ---
    skeleton = skeletonize(cleaned)

    # --- Step 6b: Medial axis skeleton (NEW) ---
    # medial_skel is the medial axis (centerline-like skeleton) of the filled binary region.
    # dist is the distance transform (distance to background), useful for debugging/pruning.
    medial_skel, dist = medial_axis(cleaned, return_distance=True)

    # --- Save results to scripts/img ---
    in_base = os.path.basename(image_path)
    name, ext = os.path.splitext(in_base)

    cleaned_u8 = (cleaned.astype(np.uint8) * 255)
    out_path_cleaned = os.path.join(os.path.dirname(image_path), f"{name}_cleaned{ext}")
    imsave(out_path_cleaned, cleaned_u8)
    print("[INFO] Saved cleaned binary result to: {}".format(out_path_cleaned))

    skeleton_u8 = (skeleton.astype(np.uint8) * 255)
    out_path_skel = os.path.join(os.path.dirname(image_path), f"{name}_skeleton_thin{ext}")
    imsave(out_path_skel, skeleton_u8)
    print("[INFO] Saved thinning skeleton result to: {}".format(out_path_skel))

    medial_u8 = (medial_skel.astype(np.uint8) * 255)
    out_path_medial = os.path.join(os.path.dirname(image_path), f"{name}_skeleton_medial{ext}")
    imsave(out_path_medial, medial_u8)
    print("[INFO] Saved medial axis skeleton result to: {}".format(out_path_medial))

    # Optional: save distance map for debugging (normalized to 0..255)
    # This helps you see stroke thickness structure and where spurs might appear.
    if dist.max() > 0:
        dist_u8 = (dist / dist.max() * 255.0).astype(np.uint8)
        out_path_dist = os.path.join(os.path.dirname(image_path), f"{name}_dist{ext}")
        imsave(out_path_dist, dist_u8)
        print("[INFO] Saved distance transform (debug) to: {}".format(out_path_dist))

    # --- Visualization (2 rows x 3 cols) ---
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), sharex=True, sharey=True)
    ax = axes.ravel()

    ax[0].imshow(gray, cmap=plt.cm.gray)
    ax[0].axis("off")
    ax[0].set_title("gray", fontsize=12)

    ax[1].imshow(binary1, cmap=plt.cm.gray)
    ax[1].axis("off")
    ax[1].set_title("binary1 (gray < t1)", fontsize=12)

    ax[2].imshow(tophat, cmap=plt.cm.gray)
    ax[2].axis("off")
    ax[2].set_title("white top-hat (on 1-gray)", fontsize=12)

    ax[3].imshow(binary2, cmap=plt.cm.gray)
    ax[3].axis("off")
    ax[3].set_title("binary2 (tophat > TH_T)", fontsize=12)

    ax[4].imshow(cleaned, cmap=plt.cm.gray)
    ax[4].axis("off")
    ax[4].set_title("cleaned (open+close+open)", fontsize=12)

    # Show medial axis skeleton in the last panel (as requested)
    ax[5].imshow(medial_skel, cmap=plt.cm.gray)
    ax[5].axis("off")
    ax[5].set_title("medial axis skeleton", fontsize=12)

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
