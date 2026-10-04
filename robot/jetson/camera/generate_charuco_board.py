#!/usr/bin/env python3
"""Generate the exact ChArUco board used by the robot camera calibration."""

import argparse
import os

import cv2

from charuco_config import DICTIONARY_NAME
from charuco_config import MARKER_LENGTH_M
from charuco_config import SQUARE_LENGTH_M
from charuco_config import SQUARES_X
from charuco_config import SQUARES_Y
from charuco_config import validate_board


def make_board(squares_x, squares_y, square_length_m, marker_length_m):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICTIONARY_NAME))
    return cv2.aruco.CharucoBoard_create(
        squares_x, squares_y, square_length_m, marker_length_m, dictionary
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", help="output PNG path")
    parser.add_argument("--pixels-per-square", type=int, default=400)
    parser.add_argument("--squares-x", type=int, default=SQUARES_X)
    parser.add_argument("--squares-y", type=int, default=SQUARES_Y)
    parser.add_argument("--square-length-m", type=float, default=SQUARE_LENGTH_M)
    parser.add_argument("--marker-length-m", type=float, default=MARKER_LENGTH_M)
    args = parser.parse_args()
    validate_board(args.squares_x, args.squares_y, args.square_length_m, args.marker_length_m)
    if args.pixels_per_square < 100:
        parser.error("--pixels-per-square must be at least 100")

    board = make_board(args.squares_x, args.squares_y, args.square_length_m, args.marker_length_m)
    size = (args.squares_x * args.pixels_per_square, args.squares_y * args.pixels_per_square)
    image = board.draw(size, marginSize=0, borderBits=1)
    parent = os.path.dirname(os.path.abspath(args.output))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)
    if not cv2.imwrite(args.output, image):
        raise RuntimeError("failed to write {0}".format(args.output))
    print("wrote {0} ({1}x{2}px)".format(args.output, image.shape[1], image.shape[0]))
    print("print at 125x175 mm (100% scale); do not fit or crop")


if __name__ == "__main__":
    main()
