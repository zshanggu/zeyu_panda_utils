#!/usr/bin/env python3
"""SpaceMouse teleop demos written straight into the SoftMimicGen HDF5 layout of the sim towel data
(e.g. hdf5_data/franka_towel_n750_seeds23.hdf5), so zeyu_openpi/examples/franka_towel/convert_towel_hdf5_to_lerobot.py
and anything else reading that file reads these too.

Cameras: agentview = the front Logitech C310 (V4L2), eye_in_hand = the wrist ZED-M (ROS 2 topics from
docker/zed_cameras.sh). Both are center-cropped to a square and resized to 512x512, sampled at --rate-hz (30). No depth.

Layout (T samples per demo; every dataset float32 unless noted, gzip like the sim file):
    data                      attrs env_args (JSON; sim_args.dt = 1/rate_hz, decimation 1, so the LeRobot converter
                              gets fps = rate_hz), total
    data/demo_N               attrs num_samples, success, source_demo, source_run
        actions               (T,7)  delta EE pose in the robot base frame, IK-relative: pos (3) and axis-angle
                                     rotation (3), both divided by IK_REL_SCALE; gripper +1 open / -1 close
        processed_actions     (T,8)  actions[:, :6] * IK_REL_SCALE (m, rad), then both finger targets (0.04 open, 0 shut)
        obs/actions           (T,7)  the previous action, zeros at t=0
        obs/agentview_image   (T,512,512,3) uint8
        obs/robot0_eye_in_hand_image (T,512,512,3) uint8
        obs/agentview_camera_pose (T,7) x y z qw qx qy qz of the front camera in the base frame (--agentview-camera-pose;
                                     NaN when not given)
        obs/robot0_eef_pos    (T,3)  O_T_EE translation (m)
        obs/robot0_eef_quat   (T,4)  w x y z, sign chosen so x >= 0 (continuous with the gripper pointing down)
        obs/robot0_gripper_qpos (T,2) finger, -finger (finger = width / 2, 0.04 open)
        obs/robot0_joint_pos_rel (T,9) 7 joints + 2 fingers, minus SIM_DEFAULT_JOINT_POS
        obs/robot0_joint_vel_rel (T,9)
        states/articulation/robot/{joint_position, joint_velocity} (T,9)  the state AFTER action t (= obs at t+1)
        states/articulation/robot/root_pose (T,7) identity [0 0 0 1 0 0 0], root_velocity (T,6) zeros
        initial_state/articulation/robot/...  (1,·) the state at obs t=0
        real/                 not in the sim file: t, wall (s), spacemouse_osc_delta (T,6) and spacemouse_gripper (T,)
                              as sent to deoxys (-1 open / +1 close, NaN before the first), t_cmd, and per camera
                              <cam>_t_frame / <cam>_frame_new
    The sim-only datasets (geoedge, instance_raw, states/deformable, initial_state/deformable) are not written.

Why actions are derived, not the SpaceMouse command: the sim's actions are IK-Rel deltas (target = current EE + 0.5 *
action, in the base frame). The SpaceMouse command is an OSC_POSE input on another scale, so instead each action is the
IK-Rel delta that takes sample t's EE pose to sample t+1's (one extra sample is recorded at the end for the last one).
Real deltas are what the arm did in 1/rate_hz s; the sim's IK lags its target, so its actions run several times larger
than its per-step motion. Scale them if you mix the two.

Usage (from zeyu_panda_utils; the NUC running auto_arm.sh and auto_gripper.sh):
    ZED_CAMERAS="zedm:zedm:15137730" docker/zed_cameras.sh          # in another terminal
    docker/run_local.sh python3 scripts/record_softmimicgen_demo.py --task fold_towel
    # test without moving the arm:
    docker/run_local.sh python3 scripts/record_softmimicgen_demo.py --task test --no-teleop --no-home --duration 5
Kept episodes are appended to --output (created if missing; an existing file gets demo_N+1, ...).
ENTER starts and stops an episode; Ctrl-C stops and keeps it, then exits.

--format picks the file type of a session (default softmimicgen, the layout above). `--format robosim` runs
record_robosim_demo.py's recorder instead (robosim_format.py): per-episode episode_NNN.h5 + front/wrist mp4s +
first-frame pngs under <workspace>/data/robosim/<task>__<time>/, with that script's own options (e.g. --no-wrist-depth,
--output-dir). Both cameras are center-cropped and resized to 512x512 as here (the wrist depth and camera_info too);
--full-frame keeps them as they come:
    docker/run_local.sh python3 scripts/record_softmimicgen_demo.py --format robosim --task franka_rope
    docker/run_local.sh python3 scripts/record_softmimicgen_demo.py --format robosim --help
--hold-orientation (hold_orientation_teleop.py) keeps the gripper vertical, twist to turn it; with either format.
"""

from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
import time
import zlib
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path

import h5py
import numpy as np
import rclpy
from termcolor import cprint

from deoxys.experimental.motion_utils import reset_joints_to
from deoxys.franka_interface import FrankaInterface
from deoxys.utils.io_devices import SpaceMouse

from log_demonstration2 import CONTROLLER_TYPE, RESET_JOINT_POSITIONS, SPACE_MOUSE_PRODUCT_ID, SPACE_MOUSE_VENDOR_ID
from record_robosim_demo import (
    DEFAULT_FRONT_DEVICE,
    RecordingError,
    Teleop,
    V4L2Camera,
    ZedRosCamera,
    enter_pressed,
    mat2quat_xyzw,
    wait_for_sensors,
)
from hold_orientation_teleop import HOLD_HELP, HoldOrientationTeleop, rotvec
from robosim_format import IMAGE_SIZE, center_square, run_robosim_recorder

IK_REL_SCALE = 0.5  # the sim's DifferentialInverseKinematicsAction scale: processed_actions = 0.5 * actions
FINGER_OPEN_M = 0.04
# obs/robot0_joint_pos_rel is relative to the sim robot's default joints (read off the sim file: states - joint_pos_rel).
SIM_DEFAULT_JOINT_POS = np.array([0.0444, -0.1894, -0.1107, -2.5148, 0.0044, 2.3775, 0.6952, 0.04, 0.04])
CAMERAS = {"agentview": "front", "eye_in_hand": "wrist"}  # dataset name -> camera
IMAGE_KEYS = {"agentview": "obs/agentview_image", "eye_in_hand": "obs/robot0_eye_in_hand_image"}
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parents[2] / "hdf5_data" / "real"  # <workspace>/hdf5_data/real


def quat_wxyz_x_positive(r: np.ndarray) -> np.ndarray:
    x, y, z, w = mat2quat_xyzw(r)
    q = np.array([w, x, y, z])
    return -q if q[1] < 0.0 else q


# -- writing ---------------------------------------------------------------


class DemoWriter:
    """One episode, staged in its own HDF5 (data/demo_0) until it is kept.

    Images are the bulk (2 x 512x512x3 at 30 Hz), so each frame is one chunk, deflated on a thread pool (zlib releases
    the GIL) and stored with write_direct_chunk: that is the HDF5 gzip filter's format, so readers see plain gzip
    datasets. Everything else is small and kept in memory until close(), when the actions and next-states are derived.
    """

    BLOCK = 64

    def __init__(self, path: Path, gzip_level: int, pool: ThreadPoolExecutor):
        self.path = path
        self._level, self._pool = gzip_level, pool
        self._h5 = h5py.File(path, "w")
        self._demo = self._h5.create_group("data").create_group("demo_0")
        self._images = {
            name: self._demo.create_dataset(key, shape=(0, IMAGE_SIZE, IMAGE_SIZE, 3),
                                            maxshape=(None, IMAGE_SIZE, IMAGE_SIZE, 3), dtype=np.uint8,
                                            chunks=(1, IMAGE_SIZE, IMAGE_SIZE, 3), compression="gzip",
                                            compression_opts=gzip_level)
            for name, key in IMAGE_KEYS.items()}
        self.samples: list[dict] = []  # low-dim part of every sample
        self.n_written = 0
        self.error: BaseException | None = None
        self._queue: queue.Queue = queue.Queue()
        self._thread = threading.Thread(target=self._run, name="demo-writer", daemon=True)
        self._thread.start()

    @property
    def n(self) -> int:
        return len(self.samples)

    def put(self, sample: dict) -> None:
        if self.error is not None:
            raise RecordingError(f"writing {self.path.name} failed: {self.error}")
        futures = {name: self._pool.submit(zlib.compress, sample.pop(f"image/{name}"), self._level)
                   for name in IMAGE_KEYS}
        self._queue.put((self.n, futures))
        self.samples.append(sample)

    def backlog(self) -> int:
        return self._queue.qsize()

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            if item is None:
                return
            if self.error is not None:
                continue  # drain; the main thread is told on its next put
            index, futures = item
            try:
                for name, future in futures.items():
                    ds = self._images[name]
                    if index >= ds.shape[0]:
                        ds.resize(ds.shape[0] + self.BLOCK, axis=0)
                    ds.id.write_direct_chunk((index, 0, 0, 0), future.result())
                self.n_written = index + 1
            except BaseException as exc:  # noqa: BLE001 - reported to the main thread
                self.error = exc

    def close(self, file_attrs: dict, camera_pose: np.ndarray) -> int:
        """Finish the images, write everything else, close. Returns T (samples minus the trailing one)."""
        self._queue.put(None)
        self._thread.join()
        T = self.n - 1 if self.error is None else min(self.n, self.n_written) - 1
        if T >= 1:
            for ds in self._images.values():
                ds.resize(T, axis=0)
            self._write_lowdim(self.samples[:T + 1], camera_pose)
            for key, value in file_attrs.items():
                self._demo.attrs[key] = value
            self._demo.attrs["num_samples"] = np.int64(T)
        self._h5.close()
        if self.error is not None:
            cprint(f"❌ Writing failed after {self.n_written} samples: {self.error}", "red")
        return max(T, 0)

    def _write_lowdim(self, s: list[dict], camera_pose: np.ndarray) -> None:
        T = len(s) - 1
        col = lambda key: np.array([x[key] for x in s])  # noqa: E731
        q, dq, width, pose = col("q"), col("dq"), col("width"), col("ee_pose")
        t, cmd_gripper = col("t"), col("cmd_gripper")

        finger = np.clip(width / 2.0, 0.0, FINGER_OPEN_M)
        finger_vel = np.zeros_like(finger)
        finger_vel[1:] = np.diff(finger) / np.maximum(np.diff(t), 1e-6)
        joint_pos = np.column_stack([q, finger, finger])
        joint_vel = np.column_stack([dq, finger_vel, finger_vel])
        eef_pos = pose[:, :3, 3]
        eef_quat = np.array([quat_wxyz_x_positive(p[:3, :3]) for p in pose])

        # Gripper action: the command in force at the next sample, flipped to the sim's +1 open. Before the first
        # SpaceMouse command (or with --no-teleop), read it off the fingers instead.
        gripper = np.where(np.isnan(cmd_gripper), np.where(finger > FINGER_OPEN_M / 2, 1.0, -1.0), -cmd_gripper)
        actions = np.zeros((T, 7))
        actions[:, :3] = (eef_pos[1:] - eef_pos[:-1]) / IK_REL_SCALE
        actions[:, 3:6] = [rotvec(pose[i + 1, :3, :3] @ pose[i, :3, :3].T) / IK_REL_SCALE for i in range(T)]
        actions[:, 6] = gripper[1:]
        processed = np.column_stack([actions[:, :6] * IK_REL_SCALE,
                                     np.repeat(np.where(actions[:, 6:7] > 0, FINGER_OPEN_M, 0.0), 2, axis=1)])

        root_pose = np.tile([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], (T, 1))
        datasets = {
            "actions": actions,
            "processed_actions": processed,
            "obs/actions": np.vstack([np.zeros((1, 7)), actions[:-1]]),
            "obs/agentview_camera_pose": np.tile(camera_pose, (T, 1)),
            "obs/robot0_eef_pos": eef_pos[:T],
            "obs/robot0_eef_quat": eef_quat[:T],
            "obs/robot0_gripper_qpos": np.column_stack([finger, -finger])[:T],
            "obs/robot0_joint_pos_rel": (joint_pos - SIM_DEFAULT_JOINT_POS)[:T],
            "obs/robot0_joint_vel_rel": joint_vel[:T],
            "states/articulation/robot/joint_position": joint_pos[1:],
            "states/articulation/robot/joint_velocity": joint_vel[1:],
            "states/articulation/robot/root_pose": root_pose,
            "states/articulation/robot/root_velocity": np.zeros((T, 6)),
            "initial_state/articulation/robot/joint_position": joint_pos[:1],
            "initial_state/articulation/robot/joint_velocity": joint_vel[:1],
            "initial_state/articulation/robot/root_pose": root_pose[:1],
            "initial_state/articulation/robot/root_velocity": np.zeros((1, 6)),
        }
        for key, value in datasets.items():
            self._demo.create_dataset(key, data=np.asarray(value, dtype=np.float32), compression="gzip",
                                      compression_opts=self._level)
        real = {
            "real/t": t[:T], "real/wall": col("wall")[:T],
            "real/spacemouse_osc_delta": col("cmd_osc")[:T], "real/spacemouse_gripper": cmd_gripper[:T],
            "real/t_cmd": col("t_cmd")[:T],
        }
        for name in CAMERAS:
            real[f"real/{name}_t_frame"] = col(f"{name}_t_frame")[:T]
            real[f"real/{name}_frame_new"] = col(f"{name}_frame_new")[:T]
        for key, value in real.items():
            self._demo.create_dataset(key, data=value)

    def delete(self) -> None:
        self.path.unlink(missing_ok=True)


def append_demo(staged: Path, output: Path, env_args: dict) -> str:
    """Copy the staged demo into output as the next demo_N (chunks are copied as they are, not recompressed)."""
    with h5py.File(output, "a") as dst, h5py.File(staged, "r") as src:
        data = dst.require_group("data")
        if "env_args" in data.attrs:
            existing = json.loads(data.attrs["env_args"])
            if existing.get("sim_args", {}).get("dt") != env_args["sim_args"]["dt"]:
                raise ValueError(f"{output} was recorded at dt={existing.get('sim_args', {}).get('dt')}, "
                                 f"not {env_args['sim_args']['dt']}; use another --output")
        else:
            data.attrs["env_args"] = json.dumps(env_args)
        indices = [int(k.split("_")[1]) for k in data.keys() if k.startswith("demo_")]
        name = f"demo_{max(indices) + 1 if indices else 0}"
        src.copy(src["data/demo_0"], data, name=name)
        data[name].attrs["source_demo"] = np.int64(int(name.split("_")[1]))
        data.attrs["total"] = np.int64(sum(int(data[k].attrs["num_samples"]) for k in data.keys()))
    return name


# -- sampling --------------------------------------------------------------


class Sampler:
    """One sample: robot state, the last SpaceMouse command, and both cameras' latest frames cropped to 512x512."""

    def __init__(self, robot: FrankaInterface, teleop: Teleop, teleop_t0: float, cameras: dict, max_frame_age: float):
        self._robot, self._teleop, self._cameras, self._max_age = robot, teleop, cameras, max_frame_age
        self._teleop_t0 = teleop_t0  # Teleop stamps t_cmd from this; samples are stamped from the episode start
        self._t0 = time.monotonic()
        self._last_seq: dict[str, int] = {}
        self._last_state = None
        self._state_changed_at = time.monotonic()

    def reset(self, t0: float) -> None:
        self._t0 = t0
        self._last_seq.clear()

    def sample(self) -> dict:
        now = time.monotonic()
        state = self._robot.last_state
        width = self._robot.last_gripper_q
        if state is None or width is None:
            raise RecordingError("no robot state (is deoxys running on the NUC?)")
        if state is not self._last_state:
            self._last_state, self._state_changed_at = state, now
        elif now - self._state_changed_at > self._max_age:
            raise RecordingError(f"robot state is {now - self._state_changed_at:.1f} s old")

        osc_delta, gripper_cmd, t_cmd = self._teleop.last()
        out = {
            "t": now - self._t0, "wall": time.time(),
            "q": np.array(state.q, dtype=np.float64), "dq": np.array(state.dq, dtype=np.float64),
            "width": float(width),
            "ee_pose": np.array(state.O_T_EE, dtype=np.float64).reshape(4, 4).T,
            "cmd_osc": osc_delta, "cmd_gripper": gripper_cmd,
            "t_cmd": t_cmd + self._teleop_t0 - self._t0,  # NaN before the first command
        }
        for name, cam_key in CAMERAS.items():
            cam = self._cameras[cam_key]
            latest = cam.latest()
            if latest is None:
                raise RecordingError(f"no frame from the {cam_key} camera" + (f" ({cam.error})" if cam.error else ""))
            rgb, t_frame, seq = latest
            if now - t_frame > self._max_age:
                raise RecordingError(f"the {cam_key} camera's last frame is {now - t_frame:.1f} s old"
                                     + (f" ({cam.error})" if cam.error else ""))
            out[f"image/{name}"] = center_square(rgb).tobytes()
            out[f"{name}_t_frame"] = t_frame - self._t0
            out[f"{name}_frame_new"] = seq != self._last_seq.get(name)
            self._last_seq[name] = seq
        return out


def record_episode(writer: DemoWriter, sampler: Sampler, rate_hz: float, duration: float | None) -> str:
    """Sample at rate_hz until ENTER, --duration or Ctrl-C (plus one trailing sample). Returns why it stopped."""
    period = 1.0 / rate_hz
    t0 = time.monotonic()
    sampler.reset(t0)
    next_t, late, last_status, n_at_status = t0, 0, t0, 0
    reason = "enter"
    cprint("🎬 Recording - press ENTER to stop", "green")
    try:
        while True:
            if enter_pressed():
                break
            now = time.monotonic()
            if duration is not None and now - t0 >= duration:
                reason = "duration"
                break
            if now < next_t:
                time.sleep(next_t - now)
            next_t += period
            if time.monotonic() - next_t > period:  # fell more than a sample behind: skip ahead, don't burst
                late += 1
                next_t = time.monotonic() + period
            writer.put(sampler.sample())

            now = time.monotonic()
            if now - last_status >= 1.0:
                last = writer.samples[-1]
                hz = (writer.n - n_at_status) / (now - last_status)
                ages = "  ".join(f"{name} {1000 * (last['t'] - last[f'{name}_t_frame']):3.0f} ms" for name in CAMERAS)
                print(f"\r   {last['t']:6.1f} s  {writer.n:5d} samples  {hz:4.1f} Hz  | {ages} "
                      f"| gripper {1000 * last['width']:4.1f} mm | write queue {writer.backlog():3d}   ",
                      end="", flush=True)
                last_status, n_at_status = now, writer.n
    except RecordingError as exc:
        print()
        cprint(f"❌ {exc}. Stopping this episode.", "red")
        reason = "error"
    except KeyboardInterrupt:
        reason = "interrupt"
    print()
    if late:
        cprint(f"⚠️  The sampling loop fell behind {late} times.", "yellow")
    return reason


FORMATS = ("softmimicgen", "robosim")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--format", choices=FORMATS, default="softmimicgen",
                        help="softmimicgen: this layout, one .hdf5 per session; robosim: record_robosim_demo.py's "
                             "episode_NNN.h5 + mp4s (see above)")
    parser.add_argument("--task", required=True, help="task name, used for the default output file name")
    parser.add_argument("--output", type=Path, default=None,
                        help="HDF5 file to append demos to (default: <workspace>/hdf5_data/real/<task>_<time>.hdf5)")
    parser.add_argument("--env-name", default="Real-Fold-Towel-Franka-IK-Rel-v0", help="env_args.env_name")
    parser.add_argument("--rate-hz", type=float, default=30.0, help="samples per second (images and state)")
    parser.add_argument("--front-device", default=DEFAULT_FRONT_DEVICE, help="V4L2 device of the front webcam")
    parser.add_argument("--front-size", default="640x480", help="WxH; 640x480 is the C310's largest YUYV size")
    parser.add_argument("--wrist-camera", default="zedm", help="ZED camera name under /zed_multi")
    parser.add_argument("--agentview-camera-pose", type=float, nargs=7, metavar=("X", "Y", "Z", "QW", "QX", "QY", "QZ"),
                        default=None, help="front camera pose in the robot base frame, for obs/agentview_camera_pose")
    parser.add_argument("--gzip-level", type=int, default=4, choices=range(10), help="4 as in the sim file")
    parser.add_argument("--max-frame-age", type=float, default=0.5,
                        help="stop an episode if a camera or the robot is silent this long (s)")
    parser.add_argument("--no-teleop", action="store_true", help="record without commanding the arm")
    parser.add_argument("--lock-rotation", nargs="*", choices=sorted(Teleop.LOCKABLE_ROTATIONS), default=None,
                        help="SpaceMouse rotations to ignore (default: pitch, or none with --hold-orientation); the "
                             "flag alone locks none")
    parser.add_argument("--hold-orientation", action="store_true",
                        help=HOLD_HELP)
    parser.add_argument("--no-home", action="store_true", help="do not move to the reset pose before each episode")
    parser.add_argument("--duration", type=float, default=None, help="stop each episode after this many seconds")
    parser.add_argument("--interface-cfg", type=Path,
                        default=Path(__file__).resolve().parent.parent / "configs" / "charmander.yml")
    parser.add_argument("--vendor-id", type=int, default=SPACE_MOUSE_VENDOR_ID)
    parser.add_argument("--product-id", type=int, default=SPACE_MOUSE_PRODUCT_ID)
    args = parser.parse_args()
    if args.lock_rotation is None:
        args.lock_rotation = [] if args.hold_orientation else ["pitch"]

    front_w, front_h = (int(v) for v in args.front_size.lower().split("x"))
    home = not args.no_home
    if home and args.no_teleop:
        cprint("Note: --no-teleop still homes the arm before each episode; add --no-home to keep it still.", "yellow")
    session = f"{args.task}_{datetime.now():%Y%m%d_%H%M%S}"
    output = args.output or DEFAULT_OUTPUT_DIR / f"{session}.hdf5"
    camera_pose = np.full(7, np.nan) if args.agentview_camera_pose is None else np.array(args.agentview_camera_pose)
    env_args = {"env_name": args.env_name, "type": 2,
                "sim_args": {"dt": 1.0 / args.rate_hz, "decimation": 1, "render_interval": 1, "num_envs": 1},
                "real": {"robot": "panda", "controller_type": CONTROLLER_TYPE, "teleop": "spacemouse",
                         "ik_rel_scale": IK_REL_SCALE, "image_size": IMAGE_SIZE,
                         "agentview": f"Logitech C310 {args.front_device}, center square crop",
                         "eye_in_hand": f"ZED {args.wrist_camera} rgb, center square crop",
                         "reset_joint_positions": list(RESET_JOINT_POSITIONS),
                         "hold_orientation": bool(args.hold_orientation)}}

    device = None
    if not args.no_teleop:
        try:
            device = SpaceMouse(vendor_id=args.vendor_id, product_id=args.product_id)
            device.start_control()
        except OSError as exc:
            cprint(f"❌ Cannot open the SpaceMouse: {exc} (see README_SETUP.md)", "red")
            return 1

    rclpy.init(args=None)
    cameras = {"front": V4L2Camera(args.front_device, front_w, front_h),
               "wrist": ZedRosCamera(args.wrist_camera, record_depth=False)}
    robot = FrankaInterface(str(args.interface_cfg), use_visualizer=False)
    t_origin = time.monotonic()
    teleop = HoldOrientationTeleop(robot, device, t_origin, lock_rotation=tuple(args.lock_rotation),
                                   hold_orientation=args.hold_orientation)
    sampler = Sampler(robot, teleop, t_origin, cameras, args.max_frame_age)
    pool = ThreadPoolExecutor(max_workers=4, thread_name_prefix="deflate")

    cprint("\n" + "=" * 64, "cyan")
    cprint("🤖 SoftMimicGen-format demo recorder", "cyan", attrs=["bold"])
    cprint(f"   task     {args.task}\n   output   {output}\n   rate     {args.rate_hz:g} Hz\n"
           f"   images   {IMAGE_SIZE}x{IMAGE_SIZE}: agentview <- {args.front_device}, "
           f"eye_in_hand <- /zed_multi/{args.wrist_camera}\n"
           f"   teleop   {'off (--no-teleop)' if args.no_teleop else 'SpaceMouse'}"
           f"{'' if args.no_teleop else ', locked: ' + (' '.join(args.lock_rotation) or 'none')}"
           f"{', orientation held' if args.hold_orientation and not args.no_teleop else ''}", "white")
    if args.agentview_camera_pose is None:
        cprint("   obs/agentview_camera_pose will be NaN (no --agentview-camera-pose)", "yellow")
    cprint("=" * 64, "cyan")

    exit_code = 0
    n_kept = 0
    try:
        cprint("⏳ Waiting for the robot and cameras...", "cyan")
        if not wait_for_sensors(robot, cameras):
            return 1
        cprint("✅ Robot and cameras streaming", "green")
        output.parent.mkdir(parents=True, exist_ok=True)
        staged = output.with_name(f".{output.stem}.episode.h5")

        while True:
            cprint(f"\n{'=' * 64}\n📝 Episode {n_kept}\n{'=' * 64}", "cyan")
            if home:
                cprint("🔄 Homing (gripper opens)...", "yellow")
                with teleop.control_lock:
                    reset_joints_to(robot, RESET_JOINT_POSITIONS, gripper_open=True)
                    teleop.hold_current()
            if args.duration is None:
                input("▶️  Press ENTER to START recording...")
            writer = DemoWriter(staged, args.gzip_level, pool)
            try:
                reason = record_episode(writer, sampler, args.rate_hz, args.duration)
            finally:
                T = writer.close({"success": np.bool_(True), "source_demo": np.int64(-1), "source_run": session},
                                 camera_pose)
            cprint(f"⏹️  Stopped ({reason}): {T} samples, {T / args.rate_hz:.1f} s", "yellow")
            if T < 1:
                writer.delete()
                cprint("Nothing recorded; no files kept.", "yellow")
            elif reason not in ("interrupt", "duration") and \
                    input("❓ Keep this episode? [Y/n]: ").strip().lower().startswith("n"):
                writer.delete()
                cprint("🗑️  Deleted.", "yellow")
                continue  # same episode number again
            else:
                name = append_demo(staged, output, env_args)
                writer.delete()
                n_kept += 1
                cprint(f"💾 Saved {T} samples as data/{name} in {output.name}", "green")
            if reason == "error":
                exit_code = 1
            if reason in ("interrupt", "duration") or \
                    not input("❓ Record another episode? [y/N]: ").strip().lower().startswith("y"):
                break
    except KeyboardInterrupt:
        cprint("\n⏹️  Interrupted", "yellow")
    finally:
        teleop.close()
        for cam in cameras.values():
            cam.close()
        try:
            robot.close()
        except Exception:  # noqa: BLE001 - never mask the original failure during cleanup
            pass
        pool.shutdown(wait=False)
        rclpy.try_shutdown()
        if output.exists():
            cprint(f"\n🏁 {output}", "green", attrs=["bold"])
    return exit_code


def run() -> int:
    """Dispatch on --format before either recorder parses its own options."""
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--format", choices=FORMATS, default="softmimicgen")
    known, rest = pre.parse_known_args()
    if known.format == "robosim":
        return run_robosim_recorder(rest)
    return main()


if __name__ == "__main__":
    sys.exit(run())
