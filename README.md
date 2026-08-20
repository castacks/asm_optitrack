# asm_optitrack

**AirStack module** (`optitrack`, type `ros_package`, targets `robot` +
`isaac-sim`) — OptiTrack motion-capture integration for AirStack: a robot-side
NatNet client whose pose stream feeds **PX4 EKF2 external-vision fusion**
(mocap becomes the vehicle's position source), and a Motive-compatible
**NatNet server emulator** for Isaac Sim so the identical robot-side stack can
be developed and CI-tested entirely in simulation.

Extracted from AirStack trunk (the OptiTrack PR series #359/#374/#375/#376)
with git history preserved, per RFC #379 ("Modular AirStack").

Data path (sim and real robot are the same from `natnet_ros2` onward):

```text
Motive (real) / in-sim NatNet emulator
  → natnet_ros2 (NatNet SDK client → PoseStamped/PoseWithCovarianceStamped)
  → vision_pose_converter → /{robot}/interface/mavros/vision_pose/pose_cov
  → PX4 EKF2 (external vision; GPS/baro/range aiding off)
```

## What's inside

| Component | Path | Role |
| --- | --- | --- |
| NatNet client node | `natnet_ros2/` (C++ colcon package) | Connects to Motive/emulator via the NatNet SDK, publishes per-rigid-body pose topics from `config/natnet_config.yaml` profiles (selected by `ROBOT_NAME`) |
| MAVROS bridges | `natnet_ros2/launch/` + converters | `vision_pose_converter`, `mavros_gp_origin`, `px4_param_setter` — turn the mocap pose into EKF2 external-vision input and set the FCU parameters |
| NatNet server emulator | `exts/optitrack.natnet.emulator/` (Kit extension) | Motive-compatible unicast NatNet server driven by Isaac Sim prims — the main value of the OptiTrack PRs: full mocap development without hardware |
| Pegasus launch scripts | `launch_scripts/one_px4_pegasus_natnet.py`, `multi_px4_pegasus_natnet.py` | Trunk's PX4 Pegasus scenes plus authored NatNet interface prims (drone body/bodies + static `Target`) |
| Tests | `tests/integration/natnet/`, `tests/system/test_optitrack_e2e.py` | Host emulator ↔ containerized client integration; full EV-fusion flight e2e (`optitrack` mark) |
| Test stack | `test_stack/` | CI target + living install documentation (RFC #379 §5) |
| Docs | `docs/px4_external_vision.md`, `docs/natnet_emulator.md` | Setup + schema guides (also embedded on the trunk docs site via `module.yaml` `docs.dir`) |
| Agent skill | `.agents/skills/optitrack-development/` | Workflow guide for agents working on this module |

## Install

```bash
cd ~/AirStack
airstack module add git@github.com:castacks/asm_optitrack.git
airstack module sync
```

**NatNet SDK note:** `natnet_ros2` builds against the proprietary OptiTrack
NatNet SDK. The `hooks.host_setup` hook
(`natnet_ros2/scripts/download-natnet-sdk.sh`, run by `module add` /
`airstack setup`) downloads it **host-side** into `natnet_ros2/lib/` +
`natnet_ros2/include/natnet/` after an EULA prompt
(`NATNET_ACCEPT_LICENSE=1` for CI). The SDK is **never tracked in git and
never baked into Docker images** — CI has a guard job that fails if any
`libNatNet`/`NatNetSDK` file is ever committed.

Then bring up the simulated mocap stack and fly it — see
[test_stack/README.md](test_stack/README.md):

```bash
ISAAC_SIM_USE_STANDALONE=true \
ISAAC_SIM_SCRIPT_NAME=modules/asm_optitrack/one_px4_pegasus_natnet.py \
PX4_PARAM_SET=external-vision \
AIRSTACK_STACK_DIR=/root/AirStack/modules/asm_optitrack/test_stack \
  airstack up --sim isaac --robots 1 --play --wait
airstack test -m optitrack --sim isaacsim --num-robots 1 -v
```

For a real robot, point `NATNET_SERVER_IP` (or the `server_ip` launch arg —
the arg wins when passed) at the Motive host and see
[docs/px4_external_vision.md](docs/px4_external_vision.md).

## Test matrix

| Mark | What it verifies | Needs |
| --- | --- | --- |
| `build_packages` | `natnet_ros2` colcon build (incl. gtest logic tests) inside the robot container | Docker |
| `integration` | Host-side emulator ↔ containerized `natnet_ros2_node`: pose round-trip, rates, transmission types | Docker (no GPU/sim) |
| `liveliness` | Stack bring-up health with the module overlaid | Docker, GPU, Isaac license |
| `optitrack` | Full e2e: emulator stream ≥ 5 Hz → EKF2 accepts EV (`EKF2_EV_CTRL=11`, GPS off) → arm → takeoff → Circle → land | Docker, GPU, Isaac license |

CI runs these through trunk's reusable `module-system-tests.yml` (the module
repo copies no test infrastructure); a weekly canary tracks trunk drift.

## Maintainer

`module.yaml` currently lists `maintainers@theairlab.org` as a **placeholder**
— to be replaced with the natnet/emulator maintainer's address before
registration (RFC #379 §8 requires a named person).
