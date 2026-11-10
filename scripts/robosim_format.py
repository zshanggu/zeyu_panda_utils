"""`record_softmimicgen_demo.py --format robosim`: record_robosim_demo.py's recorder, run unchanged, with two additions
made from outside (record_robosim_demo.py itself is not modified):

  * center crop + resize (default; --full-frame turns it off): both cameras are cropped to a centered square on their
    shorter side and resized to IMAGE_SIZE x IMAGE_SIZE (512, as the SoftMimicGen format; front C310 640x480 -> crop
    480 -> 512, wrist ZED 672x376 -> crop 376 -> 512). Everything written is transformed the same way --
    cameras/<cam>/rgb, the wrist depth (same window, nearest-neighbour so no depth is invented at edges; it stays
    registered), the mp4 clips and first-frame pngs -- and the wrist camera_info (k, p) is shifted by the crop offset
    and scaled by the resize so it still describes the stored image. Each camera group gets attrs center_crop
    ([x0, y0, size] in the full frame) and image_size.
  * --hold-orientation (hold_orientation_teleop.py): the gripper is kept vertical; twist the SpaceMouse to turn it.

The episode files record both: attrs center_crop (bool) and teleop_hold_orientation (bool).
"""

from __future__ import annotations

import json
import sys

import cv2
import numpy as np
from termcolor import cprint

import record_robosim_demo
from hold_orientation_teleop import HoldOrientationTeleop

HOLD_FLAG = "--hold-orientation"
FULL_FRAME_FLAG = "--full-frame"
IMAGE_SIZE = 512


def center_crop_window(width: int, height: int) -> tuple[int, int, int]:
    """(x0, y0, size) of the centered square on the shorter side."""
    size = min(width, height)
    return (width - size) // 2, (height - size) // 2, size


def center_square(image: np.ndarray, size: int = IMAGE_SIZE, interpolation: int | None = None) -> np.ndarray:
    """Center-crop to a square on the shorter side, then resize to size x size (area when shrinking, linear when
    enlarging, unless interpolation is given -- cv2.INTER_NEAREST for depth)."""
    x0, y0, s = center_crop_window(image.shape[1], image.shape[0])
    crop = image[y0:y0 + s, x0:x0 + s]
    if s == size:
        return np.ascontiguousarray(crop)
    if interpolation is None:
        interpolation = cv2.INTER_AREA if s > size else cv2.INTER_LINEAR
    return cv2.resize(crop, (size, size), interpolation=interpolation)


def crop_camera_info(info: dict | None, width: int, height: int, size: int = IMAGE_SIZE) -> dict | None:
    """camera_info of the full frame -> of its center square resized to size: the principal point moves by the crop
    offset, then focal lengths and principal point scale by size / square (pixel centres: c' = (c + 0.5) * f - 0.5)."""
    if info is None:
        return None
    x0, y0, square = center_crop_window(width, height)
    scale = size / square
    info = json.loads(json.dumps(info))
    info["width"] = info["height"] = size
    for key, (fx, cx, fy, cy) in (("k", (0, 2, 4, 5)), ("p", (0, 2, 5, 6))):
        m = info.get(key, [])
        if len(m) not in (9, 12):
            continue
        m[fx] *= scale
        m[fy] *= scale
        m[cx] = (m[cx] - x0 + 0.5) * scale - 0.5
        m[cy] = (m[cy] - y0 + 0.5) * scale - 0.5
        if key == "p":  # P's translation column (stereo baseline term) is in pixels too
            m[3] *= scale
            m[7] *= scale
    return info


class CroppedV4L2Camera(record_robosim_demo.V4L2Camera):
    def latest(self):
        frame = super().latest()
        return None if frame is None else (center_square(frame[0]), *frame[1:])

    def attrs(self) -> dict:
        return {**super().attrs(), "center_crop": list(center_crop_window(self.width, self.height)),
                "image_size": IMAGE_SIZE}


class CroppedZedRosCamera(record_robosim_demo.ZedRosCamera):
    def latest(self):
        frame = super().latest()
        return None if frame is None else (center_square(frame[0]), *frame[1:])

    def latest_depth(self):
        depth = super().latest_depth()
        return None if depth is None else (center_square(depth[0], interpolation=cv2.INTER_NEAREST), *depth[1:])

    def attrs(self) -> dict:
        attrs = super().attrs()
        info = self.camera_info
        if info is not None:
            w, h = int(info["width"]), int(info["height"])
            attrs["camera_info"] = json.dumps(crop_camera_info(info, w, h))
            attrs["center_crop"] = list(center_crop_window(w, h))
            attrs["image_size"] = IMAGE_SIZE
        return attrs


def run_robosim_recorder(argv: list[str]) -> int:
    """record_robosim_demo.main() on argv, plus the center crop (unless --full-frame) and --hold-orientation."""
    hold = HOLD_FLAG in argv
    crop = FULL_FRAME_FLAG not in argv
    if any(a in ("-h", "--help") for a in argv):
        cprint(f"--format robosim also takes {FULL_FRAME_FLAG} (record the cameras as they come; default: center "
               f"square crop resized to {IMAGE_SIZE}x{IMAGE_SIZE}) and {HOLD_FLAG} (keep the gripper vertical; twist "
               f"the SpaceMouse to turn it).", "cyan")
    argv = [a for a in argv if a not in (HOLD_FLAG, FULL_FRAME_FLAG)]
    if hold and not any(a == "--lock-rotation" or a.startswith("--lock-rotation=") for a in argv):
        argv.append("--lock-rotation")  # the flag alone: nothing locked; the hold decides what turns the gripper

    teleops: list[HoldOrientationTeleop] = []

    def make_teleop(robot, device, t0, lock_rotation=("pitch",)):
        teleops.append(HoldOrientationTeleop(robot, device, t0, lock_rotation, hold_orientation=hold))
        return teleops[-1]

    make_teleop.LOCKABLE_ROTATIONS = record_robosim_demo.Teleop.LOCKABLE_ROTATIONS  # main() reads it for choices
    reset_joints_to = record_robosim_demo.reset_joints_to

    def reset_then_hold(*args, **kwargs):
        result = reset_joints_to(*args, **kwargs)
        for teleop in teleops:
            teleop.hold_current()
        return result

    class Writer(record_robosim_demo.EpisodeWriter):
        def __init__(self, stem, rate_hz, file_attrs, camera_attrs):
            super().__init__(stem, rate_hz, {**file_attrs, "teleop_hold_orientation": hold, "center_crop": crop},
                             camera_attrs)

    patches = {"Teleop": make_teleop, "reset_joints_to": reset_then_hold, "EpisodeWriter": Writer}
    if crop:
        patches.update(V4L2Camera=CroppedV4L2Camera, ZedRosCamera=CroppedZedRosCamera)
    saved = {name: getattr(record_robosim_demo, name) for name in patches}
    for name, value in patches.items():
        setattr(record_robosim_demo, name, value)
    try:
        sys.argv = [record_robosim_demo.__file__] + argv
        notes = ([f"cameras center-cropped and resized to {IMAGE_SIZE}x{IMAGE_SIZE}"] if crop else []) + \
                (["gripper kept vertical (twist the SpaceMouse to turn it)"] if hold else [])
        if notes:
            cprint("robosim format: " + "; ".join(notes), "cyan")
        return record_robosim_demo.main()
    finally:
        for name, value in saved.items():
            setattr(record_robosim_demo, name, value)
