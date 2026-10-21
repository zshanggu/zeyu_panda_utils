"""Short, instrumented OSC_POSE response test.

From the shared home pose, commands a constant small +x delta for a fixed time and logs how far
the end effector actually moves, then returns home with joint control. Use it to tell "OSC does
not run at all" apart from "OSC runs but under-responds" before tuning gains.

Usage:
    python scripts/osc_probe.py                       # 2 cm delta, 2 s, default OSC config
    python scripts/osc_probe.py --delta-m 0.01 --duration-s 1.5
    python scripts/osc_probe.py --controller-cfg configs/osc-pose-tuned.yml --kp-translation 300
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
from termcolor import cprint

from deoxys.experimental.motion_utils import reset_joints_to
from deoxys.franka_interface import FrankaInterface
from deoxys.utils.config_utils import get_default_controller_config
from deoxys.utils.yaml_config import YamlConfig

PROJECT_DIR = Path(__file__).parent.parent
RESET_JOINT_POSITIONS = [0.0916, -0.1983, -0.0199, -2.4732, -0.0131, 2.3040, 0.8481]
MAX_TRAVEL_M = 0.08  # abort if the hand moves further than this from the start


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--interface-cfg", type=Path, default=PROJECT_DIR / "configs" / "charmander.yml")
    parser.add_argument("--controller-cfg", type=Path, default=None,
                        help="OSC yml (default: deoxys osc-pose-controller.yml)")
    parser.add_argument("--kp-translation", type=float, default=None)
    parser.add_argument("--delta-m", type=float, default=0.02, help="+x offset requested every step")
    parser.add_argument("--duration-s", type=float, default=2.0)
    args = parser.parse_args()

    if args.controller_cfg is None:
        osc_cfg = get_default_controller_config(controller_type="OSC_POSE")
    else:
        osc_cfg = YamlConfig(str(args.controller_cfg)).as_easydict()
    if args.kp_translation is not None:
        osc_cfg.Kp.translation = args.kp_translation
    scale = osc_cfg.action_scale.translation
    action_x = float(np.clip(args.delta_m / scale, 0.0, 1.0))

    robot = FrankaInterface(str(args.interface_cfg), use_visualizer=False)
    try:
        deadline = time.time() + 10.0
        while len(robot._state_buffer) == 0 or robot.last_gripper_q is None:
            if time.time() > deadline:
                cprint("[FAIL] no state from the NUC", "red")
                return 1
            time.sleep(0.1)

        cprint("[info] homing (joint control)", "cyan")
        reset_joints_to(robot, RESET_JOINT_POSITIONS)
        start = robot.last_eef_pose[:3, 3].copy()
        cprint(f"[info] start ee={np.round(start, 4)}  Kp.translation={osc_cfg.Kp.translation}  "
               f"delta={args.delta_m * 100:.1f} cm/step (action {action_x:.2f})", "cyan")

        t0 = time.time()
        next_print = 0.0
        while time.time() - t0 < args.duration_s:
            robot.control(controller_type="OSC_POSE", action=[action_x, 0, 0, 0, 0, 0, -1.0],
                          controller_cfg=osc_cfg)
            t = time.time() - t0
            moved = robot.last_eef_pose[:3, 3] - start
            if t >= next_print:
                print(f"  t={t:4.2f}s  dx={moved[0] * 1000:7.1f} mm  |d|={np.linalg.norm(moved) * 1000:7.1f} mm")
                next_print += 0.25
            if np.linalg.norm(moved) > MAX_TRAVEL_M:
                cprint(f"[stop] moved more than {MAX_TRAVEL_M * 100:.0f} cm", "yellow")
                break

        moved = robot.last_eef_pose[:3, 3] - start
        cprint(f"[result] after {time.time() - t0:.2f}s: dx={moved[0] * 1000:.1f} mm, "
               f"total={np.linalg.norm(moved) * 1000:.1f} mm", "green" if moved[0] > 0.005 else "yellow")

        cprint("[info] returning home (joint control)", "cyan")
        reset_joints_to(robot, RESET_JOINT_POSITIONS)
    finally:
        robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
