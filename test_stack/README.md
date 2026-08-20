# optitrack test stack

This folder is the module's CI target and its living install documentation
(RFC #379 §5): an AirStack bring-up where Isaac Sim streams Motive-compatible
NatNet mocap from the in-sim emulator, `natnet_ros2` consumes it on the robot,
and PX4 flies on EKF2 external-vision fusion — verified by trunk's existing
system-test suite (`optitrack` mark, `tests/system/test_optitrack_e2e.py` in
this repo).

**Interim form (pre-stacks):** trunk reference stack folders do not exist yet,
so `launch/stack.launch.xml` wraps trunk's monolithic `autonomy_bringup` and
adds the module's one include (unconditional — the stack replaces trunk's
`LAUNCH_NATNET` env toggle), and `docker-compose.yaml` is a manual compose
override rather than a generated module layer.

## How to run

1. Add the module to an AirStack checkout (this also runs the NatNet SDK
   host-setup hook — proprietary SDK, downloaded to `natnet_ros2/lib/` +
   `include/natnet/`, never tracked, never baked into images):

   ```bash
   cd ~/AirStack
   airstack module add git@github.com:castacks/asm_optitrack.git   # or a local path
   airstack module sync
   ```

2. Bring the stack up with the OptiTrack scene and PX4 external-vision
   parameters (mirrors trunk's `overrides/isaac-optitrack-simulation.env`,
   which stays in trunk for now):

   ```bash
   ISAAC_SIM_USE_STANDALONE=true \
   ISAAC_SIM_SCRIPT_NAME=modules/asm_optitrack/one_px4_pegasus_natnet.py \
   PX4_PARAM_SET=external-vision LAUNCH_NATNET=true \
     airstack up --sim isaac --robots 1 --play --wait
   ```

   (`ISAAC_SIM_SCRIPT_NAME` / `PX4_PARAM_SET` are compose-interpolation
   variables, so they ride the command environment; see the comments in
   `docker-compose.yaml`.)

3. Run the system tests (trunk's suite, unchanged):

   ```bash
   airstack test -m optitrack --sim isaacsim --num-robots 1 -v
   ```

   The `optitrack` mark drives the full chain: NatNet pose stream ≥ 5 Hz →
   EKF2 accepts external vision (`EKF2_EV_CTRL=11`, `EKF2_GPS_CTRL=0`) →
   arm → takeoff → Circle trajectory → land.

## Files

- `modules.repos` — vcstool pin of this module (placeholder URL/tag until the
  repo is pushed); top-level `airstack_compat` declares the tested trunk range.
- `launch/stack.launch.xml` — interim flat wrapper: trunk bringup + an
  unconditional, explicitly-wired `natnet_ros2` include (`server_ip` launch
  arg instead of ambient env).
- `docker-compose.yaml` — compose override adding the emulator extension
  mount, the scene selection, and the robot-side NatNet env.
