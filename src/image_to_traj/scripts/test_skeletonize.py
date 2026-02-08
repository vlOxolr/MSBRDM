#!/usr/bin/env python3
import os
import glob
import argparse
from typing import Optional

import numpy as np
import matplotlib.pyplot as plt

from skimage.io import imread
from skimage.color import rgb2gray
from skimage.filters import threshold_otsu
from skimage.morphology import (
    white_tophat,
    opening,
    closing,
    skeletonize,
    rectangle,
    disk,
)

from skimage.io import imsave



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

    # --- Step 1: Binarization (required) ---
    # Otsu thresholding, then foreground = (gray < t1) for dark strokes on bright background.
    # If your strokes are bright on a dark background, flip to (gray > t1).
    t1 = threshold_otsu(gray)
    binary1 = gray < t1

    # --- Step 2: White top-hat (REPLACED algorithm, required) ---
    # We replace the previous top-hat with:
    #   inv = 1.0 - gray
    #   tophat = white_tophat(inv, rectangle(TH_H, TH_W))
    #
    # Rationale:
    # - white_tophat extracts small BRIGHT structures compared to the local background.
    # - By inverting grayscale, DARK strokes become BRIGHT, so they can be enhanced by white_tophat.
    #
    # Tuning guide for TH_H / TH_W:
    # - TH_W (width) ~ stroke thickness (start around 1x to 2x of stroke width).
    # - TH_H (length) controls how strongly "strip-like" patterns are emphasized.
    #   Increase TH_H if strokes are long and continuous; decrease if you lose corners/details.
    inv = 1.0 - gray

    TH_H = 15
    TH_W = 10
    selem_tophat = rectangle(TH_H, TH_W)
    tophat = white_tophat(inv, selem_tophat)

    # --- Step 3: Binarization again (required) ---
    # After top-hat, threshold the response to obtain a stroke mask.
    #
    # Tuning guide for TH_T:
    # - Increase TH_T to reduce noise (cleaner, but may miss faint strokes).
    # - Decrease TH_T to keep more strokes (more complete, but noisier).
    # Tip: If you change TH_H/TH_W, you usually need to re-tune TH_T.
    TH_T = 0.2
    binary2 = tophat > TH_T

    # --- Step 4: Opening (required) ---
    # Tuning guide for OPEN_R:
    # - Increase OPEN_R to remove thin texture / small speckles more aggressively.
    # - Decrease OPEN_R if real strokes get eroded or broken.
    OPEN_R = 1
    opened = opening(binary2, disk(OPEN_R))

    # --- Step 5: Closing (required) ---
    # Tuning guide for CLOSE_R:
    # - Increase CLOSE_R to connect fragmented strokes and fill small gaps.
    # - Decrease CLOSE_R if distinct nearby strokes get merged.
    CLOSE_R = 2
    cleaned = closing(opened, disk(CLOSE_R))

    cleaned = opening(cleaned, disk(OPEN_R))

    # --- Step 6: Skeletonization (required) ---
    skeleton = skeletonize(cleaned)

    # --- Save skeleton result to scripts/img (required) ---
    # Save as 8-bit image (0 or 255) to ensure consistent visualization in common viewers.
    in_base = os.path.basename(image_path)
    name, ext = os.path.splitext(in_base)

    cleaned_u8 = (cleaned.astype(np.uint8) * 255)
    out_path = os.path.join(os.path.dirname(image_path), f"{name}_cleaned{ext}")
    imsave(out_path, cleaned_u8)
    print("[INFO] Saved cleaned binary result to: {}".format(out_path))

    skeleton_u8 = (skeleton.astype(np.uint8) * 255)
    out_path_skel = os.path.join(os.path.dirname(image_path), f"{name}_skeleton{ext}")
    imsave(out_path_skel, skeleton_u8)
    print("[INFO] Saved skeleton result to: {}".format(out_path_skel))

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
    ax[4].set_title("opening + closing", fontsize=12)

    ax[5].imshow(skeleton, cmap=plt.cm.gray)
    ax[5].axis("off")
    ax[5].set_title("skeleton", fontsize=12)

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
