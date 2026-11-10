"""SpaceMouse teleop that keeps the gripper vertical: --hold-orientation for the demo recorders.

record_robosim_demo.Teleop sends the puck's rotation as it is (minus --lock-rotation), so the gripper tilts with the
puck and slowly drifts even when the rotation is locked. HoldOrientationTeleop keeps the gripper at the orientation it
has after homing (the reset pose's vertical gripper):

  * tilting the puck (forward/back, left/right) is ignored, and drift is corrected on every command;
  * twisting the puck spins the gripper about its vertical axis as in plain teleop, and it then stays at the angle it
    was turned to; twists under TWIST_DEADBAND (~11% of the puck's range) are ignored, so pushing the puck sideways
    does not turn it.

How: deoxys' OSC makes its goal R(action[3:6] * action_scale.rotation) @ R_current in the base frame
(franka-interface osc_impedance.cpp ComputeGoal; rotation scale 1.0), so sending rotvec(R_hold @ R_current^T) puts the
goal on R_hold exactly, also while the puck is idle. The puck's twist is action[5] (rotation about the base z axis,
input2action's -yaw), which for a vertical gripper is a spin about its own axis; it is added to that, and R_hold's
heading follows it (re-anchored to the current heading while twisting and when the twist ends, so the gripper stops
where it is rather than running on to a target that led it). The puck's tilts (action[3:5]) are never sent.

This module only builds on record_robosim_demo, which it leaves unchanged:
  * record_softmimicgen_demo.py uses HoldOrientationTeleop directly;
  * run_robosim_recorder() runs record_robosim_demo.main() with its Teleop swapped for this one (that is what
    `record_softmimicgen_demo.py --format robosim --hold-orientation` does).
"""

from __future__ import annotations

import sys

import numpy as np
from termcolor import cprint

import record_robosim_demo
from record_robosim_demo import Teleop, mat2quat_xyzw

HOLD_FLAG = "--hold-orientation"
HOLD_HELP = ("keep the gripper vertical (its orientation after homing) while the arm moves, without drift: tilting the "
             "SpaceMouse is ignored, twisting it spins the gripper about its vertical axis, and it stays at the angle "
             "it was turned to")


def rotvec(r: np.ndarray) -> np.ndarray:
    """Rotation matrix -> axis-angle vector (angle <= pi)."""
    x, y, z, w = mat2quat_xyzw(r)  # w >= 0
    v = np.array([x, y, z])
    s = np.linalg.norm(v)
    return np.zeros(3) if s < 1e-12 else v * (2.0 * np.arctan2(s, w) / s)


def heading(r: np.ndarray) -> float:
    """Rotation about the base z axis: the angle of the gripper's x axis in the base xy plane."""
    return float(np.arctan2(r[1, 0], r[0, 0]))


class _HoldingRobot:
    """The FrankaInterface as Teleop sees it: control() rewrites the rotation of each OSC command in place (so Teleop
    records what was actually sent) before passing it on. Everything else goes straight to the real interface."""

    def __init__(self, robot, teleop: "HoldOrientationTeleop"):
        self._robot, self._teleop = robot, teleop

    def __getattr__(self, name):
        return getattr(self._robot, name)

    def control(self, controller_type, action, controller_cfg, **kwargs):
        scale = float(controller_cfg.action_scale.rotation)
        action[3:6] = self._teleop.rotation_command(np.asarray(action[3:6], dtype=np.float64) * scale) / scale
        return self._robot.control(controller_type=controller_type, action=action, controller_cfg=controller_cfg,
                                   **kwargs)


class HoldOrientationTeleop(Teleop):
    """record_robosim_demo.Teleop, plus hold_orientation (see the module docstring). Call hold_current() after
    homing; until then the orientation at the first command is held."""

    MAX_HOLD_CORRECTION_RAD = 0.3  # per command; a larger error (never expected) is closed over a few ticks
    TWIST_DEADBAND = 0.04  # rad per command; a full twist is 0.375 (input2action: 0.005 * 75), so ~11%

    def __init__(self, robot, device, t0: float, lock_rotation: tuple[str, ...] = ("pitch",),
                 hold_orientation: bool = False):
        self.hold_orientation = hold_orientation
        self.hold_target: np.ndarray | None = None  # 3x3, base frame
        self._twisting = False
        self._real_robot = robot
        # Teleop starts its thread in __init__, so the wrapper has to be in place before that.
        super().__init__(_HoldingRobot(robot, self) if hold_orientation else robot, device, t0, lock_rotation)

    def _ee_rotation(self) -> np.ndarray | None:
        state = self._real_robot.last_state
        if state is None:
            return None
        return np.array(state.O_T_EE, dtype=np.float64).reshape(4, 4).T[:3, :3]

    def hold_current(self) -> None:
        """Hold the gripper at its current orientation from now on."""
        r = self._ee_rotation()
        if r is not None:
            self.hold_target = r
            self._twisting = False

    def rotation_command(self, user_rotation: np.ndarray) -> np.ndarray:
        """The rotation to send (base frame, rad) for the puck's rotation user_rotation (base frame, rad): the twist
        (z) if any, plus what brings the gripper back to hold_target."""
        r = self._ee_rotation()
        if r is None:
            return np.zeros(3)
        if self.hold_target is None:
            self.hold_target = r
        twist = user_rotation[2] if abs(user_rotation[2]) >= self.TWIST_DEADBAND else 0.0
        if twist or self._twisting:  # twisting, or a twist just ended: hold the heading the gripper has now
            turn = heading(r) - heading(self.hold_target)
            c, s = np.cos(turn), np.sin(turn)
            self.hold_target = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ self.hold_target
            self._twisting = bool(twist)
        delta = rotvec(self.hold_target @ r.T)
        norm = np.linalg.norm(delta)
        if norm > self.MAX_HOLD_CORRECTION_RAD:
            delta *= self.MAX_HOLD_CORRECTION_RAD / norm
        delta[2] += twist
        return delta


def run_robosim_recorder(argv: list[str]) -> int:
    """record_robosim_demo.main() on argv, with --hold-orientation added from outside: its Teleop is swapped for
    HoldOrientationTeleop, the hold is re-taken after each homing, and the HDF5 gets a teleop_hold_orientation attr.
    Without the flag it is record_robosim_demo.main() as is."""
    if HOLD_FLAG not in argv:
        sys.argv = [record_robosim_demo.__file__] + argv
        return record_robosim_demo.main()
    argv = [a for a in argv if a != HOLD_FLAG]
    if not any(a == "--lock-rotation" or a.startswith("--lock-rotation=") for a in argv):
        argv.append("--lock-rotation")  # the flag alone: nothing locked; the hold decides what turns the gripper

    teleops: list[HoldOrientationTeleop] = []

    def make_teleop(robot, device, t0, lock_rotation=("pitch",)):
        teleops.append(HoldOrientationTeleop(robot, device, t0, lock_rotation, hold_orientation=True))
        return teleops[-1]

    reset_joints_to = record_robosim_demo.reset_joints_to

    def reset_then_hold(*args, **kwargs):
        result = reset_joints_to(*args, **kwargs)
        for teleop in teleops:
            teleop.hold_current()
        return result

    class Writer(record_robosim_demo.EpisodeWriter):
        def __init__(self, stem, rate_hz, file_attrs, camera_attrs):
            super().__init__(stem, rate_hz, {**file_attrs, "teleop_hold_orientation": True}, camera_attrs)

    saved = (record_robosim_demo.Teleop, record_robosim_demo.reset_joints_to, record_robosim_demo.EpisodeWriter)
    record_robosim_demo.Teleop, record_robosim_demo.reset_joints_to, record_robosim_demo.EpisodeWriter = (
        make_teleop, reset_then_hold, Writer)
    make_teleop.LOCKABLE_ROTATIONS = Teleop.LOCKABLE_ROTATIONS  # main() reads it for --lock-rotation's choices
    try:
        sys.argv = [record_robosim_demo.__file__] + argv
        cprint("--hold-orientation: the gripper is kept vertical; twist the SpaceMouse to turn it", "cyan")
        return record_robosim_demo.main()
    finally:
        record_robosim_demo.Teleop, record_robosim_demo.reset_joints_to, record_robosim_demo.EpisodeWriter = saved
