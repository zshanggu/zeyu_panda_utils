# Franka bring-up on earlgrey (Zeyu's setup)

Status as of 2026-10-20. Robot control, SpaceMouse teleop and camera streaming all work from
Docker on this PC. Demo recording is ready but not yet tested.
`README_SETUP.md` describes another user's setup (`/home/res1`, NUC at `.8`, Desk unreachable),
so don't follow its paths.

## How it fits together

```
earlgrey (this PC, 172.16.0.5)          NUC baskin3 (172.16.0.1)             Panda (172.16.0.2)
docker: panda-deoxys  --ZMQ 5555-5558-->  auto_arm.sh / auto_gripper.sh  --FCI/libfranka-->  arm + hand
  SpaceMouse (USB), ZED topics (ROS 2)    (deoxys, RT kernel, libfranka 0.9.2)
```

- **Never connect libfranka from this PC directly.** The NUC owns the FCI connection, and direct
  sessions get dropped after about 2 s. Everything goes through deoxys `FrankaInterface`.
- The robot runs Panda system 4.2.2, and the NUC's deoxys is at `~/jeremy/0.8.0/deoxys_control/deoxys`.
  A copy (Python side used here) is at `/data/zeyu/Deformable/deoxys_control`.

## Start-up checklist

1. **Desk.** Use **Chrome**, not Firefox: the robot's certificate is revoked and Firefox won't
   let you continue. Open `https://172.16.0.2/desk/`, click Advanced → Proceed (or type
   `thisisunsafe`), log in, **unlock the joints** (open-padlock button), release the stop button
   (light **blue**), then **≡ → Activate FCI**. Leave the "FCI is active" box open.
   - The "End-effector not connected" message while FCI is active is a **false alarm**. Ignore it.
2. **NUC.** Use two terminals on the NUC (user `abrar`):
   ```bash
   cd ~/jeremy/0.8.0/deoxys_control/deoxys
   ./auto_scripts/auto_arm.sh config/charmander.yml       # terminal 1
   ./auto_scripts/auto_gripper.sh config/charmander.yml   # terminal 2
   ```
   `auto_arm.sh` restarts `franka-interface` by itself when a client ends (teleop does this on exit).
3. **This PC.** Run everything through the container, from `/data/zeyu/Deformable/zeyu_panda_utils`:
   ```bash
   docker/run.sh python3 scripts/franka_manipulation.py --check   # no motion, ports only
   docker/run.sh python3 scripts/franka_manipulation.py --demo home
   ```
   If a terminal says `permission denied ... docker.sock`, run `newgrp docker` (or log out and
   back in once).

## Commands (all from `zeyu_panda_utils/`, stop button in hand)

| What | Command |
|---|---|
| Home (lab reset pose) | `docker/run.sh python3 scripts/franka_manipulation.py --demo home` |
| 10 cm square (OSC smoke test) | `docker/run.sh python3 scripts/franka_manipulation.py --demo square` |
| OSC response test (+x, prints mm moved) | `docker/run.sh python3 scripts/osc_probe.py` |
| SpaceMouse teleop | `docker/run.sh python3 scripts/panda_teleop.py --vendor-id 1133 --product-id 50726` |
| Record demos (not yet tested) | see below |

SpaceMouse: push/tilt the puck to move and rotate, **hold the left button** to close the gripper, and
press the **right button** (or Ctrl+C) to quit. A full push is fast, so start gently. The
"Fatal Python error ... daemon threads" printed on exit is harmless.

### Recording demos

```bash
docker/run.sh python3 scripts/log_demonstration2.py \
    --output_dir /data/zeyu/Deformable/data/test_demos --description towel_test \
    --recording_rate_hz 15.0 --camera_ids zed2i zedm --no-depth-cameras zedm \
    --vendor-id 1133 --product-id 50726
```
The script homes the arm, then: Enter = start, teleop, Enter = stop, y/n = record another.
Record one short demo first and check the HDF5 before collecting real data.

## Cameras

| ROS name | Camera | Serial | View |
|---|---|---|---|
| `zedm` | ZED-M | 15137730 | **wrist** (fingers visible at the bottom) |
| `zed2i` | ZED 2i | 33386243 | third-person, top/side, sees the whole table and robot base |
| `zed` | ZED (original) | n/a | third-person, front, facing the robot |

- **Start the cameras yourself with `docker/zed_cameras.sh`** (Ctrl+C stops them). It runs the `zed-ros2` image
  (`docker/zed.Dockerfile`: ZED SDK 4.2.5 + zed-ros2-wrapper humble-v4.2.5) and publishes the topics the apps use,
  `/zed_multi/<cam>/rgb/image_rect_color` (left lens), `left/` and `right/image_rect_color`, and
  `depth/depth_registered`, at 672x376 and 30 Hz. Settings are in `docker/zed_params.yaml`: native VGA, depth
  `PERFORMANCE` (`ZED_DEPTH_MODE=NEURAL` for better depth; its first start builds the AI model), positional
  tracking and camera TFs off.
  - `docker/zed_cameras.sh --list` lists the connected cameras and their serial numbers. The original ZED's
    serial goes into `ZED_CAMERAS` (the default spec has a placeholder for it).
  - The SDK downloads each camera's factory calibration once, into `/data/zeyu/Deformable/.cache/zed/settings`.
  - **Only one program can open a camera.** Before 2026-10-23 the images came from `res1`'s `launch_all.sh`
    (another Linux user). It stopped when the cameras were re-plugged. `zed_cameras.sh` refuses to start while
    `/zed_multi` topics are already published, so agree with that user on who runs the cameras.
- ROS 2 between containers and users needs UDP. `docker/fastdds_udp_only.xml` handles this, and `zed_cameras.sh`,
  `run.sh` and the deploy app all set it. Without it you see topics but receive no images.
- `logs/camera_id/*.jpg` holds one sample frame from each camera.

## Known issues and fixes

- **Arm ignores OSC commands** (it doesn't move but the gripper still responds, and the NUC terminal ends with
  `Joint position callback` → `Initialize OSC Pose` → `Initialize Pose interpolator!`): `franka-interface`
  hung while switching from a joint move to OSC within one process. This is intermittent and was seen twice. **Restart
  `auto_arm.sh`.** `zeyu-Policy-deploy-Franka` avoids it by restarting the NUC process after every joint move
  (`backend/robot.py: restart_nuc_controller`). The zeyu_panda_utils scripts (`franka_manipulation.py`,
  `osc_probe.py`, `log_demonstration2.py`) still home and then switch, so they can hit it. After a restart,
  OSC moved about 55 mm/s with a 2 cm-per-step command.
- OSC isn't exact: about 26 mm of sideways drift over a 52 mm move, and `move_to_ee_position` stops within 1 cm.
- The NUC file log (`logs/deoxys_control_arm_program.log`) is buffered and lags, so the live terminal
  is more reliable. To filter the log:
  `grep -v -E 'Counter [0-9]+$|No traj interpolator|No state estimator' logs/deoxys_control_arm_program.log | tail -40`
- The firewall (ufw) is inactive. A rule allowing the NUC on ports 5555 and 5557 was added in case it is enabled later.

## What was changed

- `zeyu_panda_utils/docker/Dockerfile`, `docker/run.sh`, `docker/fastdds_udp_only.xml`: the `panda-deoxys`
  image (ROS 2 Humble + deoxys deps, protobuf 3.20). `run.sh` mounts `/data/zeyu/Deformable` and passes
  in the SpaceMouse, which is found again on every start. Rebuild with
  `docker build -t panda-deoxys -f docker/Dockerfile docker`.
- `zeyu_panda_utils/scripts/osc_probe.py`: new OSC response test.
- `zeyu_panda_utils/scripts/panda_teleop.py`: the right button now quits instead of crashing.
- `deoxys_control/deoxys/deoxys/utils/io_devices/spacemouse.py`: supports the older SpaceNavigator
  (`046d:c626`), which sends translation and rotation as separate reports. This is the local copy only;
  the NUC is unchanged.
- Obsolete and safe to delete: `zeyu_openpi/examples/franka_real/` and the `franka-real` Docker image.
  These were from the early attempt at direct libfranka control.

## Pi0.5 deployment

`/data/zeyu/Deformable/zeyu-Policy-deploy-Franka` (see its README): `./run.sh` starts the Pi0.5 policy server
(GPU container) and a web UI on http://localhost:8082 with live side and wrist views, Connect / Home / Run /
Pause / Step / Stop, safety limits, checkpoint switching and episode recording. It was tested end to end without
the robot (live cameras → policy → actions, about 90 ms per chunk after a 10 s first call). It has not yet run on
the arm.

## Next steps

1. Record a test demo and inspect the HDF5 (images, state, actions, gripper).
2. Decide the scene camera (`zed2i` is suggested) and collect real demos.
3. Convert the demos to LeRobot (see `zeyu_openpi/examples/franka_towel/`) and fine-tune Pi0.5.
4. Write the Pi0.5 robot client: openpi websocket client + `FrankaInterface` OSC_POSE deltas at 20 Hz
   + ZED images. `scripts/act_inference.py` / `rm_bc_inference.py` are templates.
5. Done 2026-10-23: our own ZED camera container (`docker/zed_cameras.sh`), so we don't depend on `res1`.
