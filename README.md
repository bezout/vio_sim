# VioSim

VioSim is a standalone C++17 simulator for synchronized cameras and an IMU,
released under the included MIT license. It provides the `viosim_core` library,
the `viosim` dataset generator and the `viosim_lab_preview` application.

It generates two or four synchronized grayscale images, a SEUCM or UCM camera
calibration, noisy and clean IMU streams, ground truth, and dataset metadata.
The simulator does not depend on any SLAM implementation.

## GPU ray cache, postprocessing, and profiling

The GPU computes camera rays once when the renderer starts, using the same
UCM/SEUCM shader formulas as the reference path. The table is shared by all
cameras, and stores the two ray differences used by mipmapping when needed.
It is independent of poses, trajectory, noise and recording length. Changing
calibration, resolution or supersampling requires constructing a new renderer.
At 640x400 it uses 11.72 MiB with `samples: 1` and mipmapping, or 46.88 MiB with
`samples: 2`. Without mipmapping, only the ray directions are stored (one third
of that memory). CPU rendering already precomputes its rays.

```yaml
gpu_precompute_rays: true # Default; false retains the reference path for A/B tests
profile_render: true     # Default false; available in live mode and offline exports
gpu_postprocess: true    # Default; blur, contrast, noise and 8-bit conversion on GPU
```

Offline generation prints the profile directly to the terminal.

GPU postprocessing uses two separable Gaussian passes with the same kernel and
reflect-101 boundaries as the CPU. The second pass applies contrast, brightness,
Gaussian read noise and rounding, and stores R8UI pixels. Readback therefore
transfers one byte per pixel instead of four. Only image packaging remains on
the CPU. The profile's GPU wait includes these extra passes.
The image noise preserves the SplitMix64 stream and Box-Muller pairing of the
CPU implementation, indexed by pixel for parallel execution. It remains
deterministic for fixed seeds on the same GPU/driver. Float arithmetic can cause
rare one-level grayscale differences from CPU output; cross-device bitwise
identity is not guaranteed. IMU noise is unchanged. Select `gpu_postprocess: false`
to keep the original CPU postprocessing and float readback for comparisons.
These three options affect only the GPU backend; CPU output is unchanged.

Profiling prints cumulative average/maximum milliseconds per camera group:
`mutex_wait`, host `submit`, host `gpu_wait_host`,
`readback`, `post_cpu`, and `total_host`. `post_cpu` includes image packaging and,
when GPU postprocessing is disabled, float copies, blur, noise and conversion.
Host durations measure elapsed time,
including OS scheduling; they are not CPU utilization measurements.
`gpu_wait_host` measures waiting for completion after submission, including
driver and OS scheduling; it is not a pure shader execution time. Hardware
timer queries were not retained because this driver's compute timestamps were
inconsistent with completion times. `total_host` already includes all stages.
The profiler explicitly finishes GPU work before readback to separate waiting
from the transfer; this extra synchronization is absent when profiling is off.
`ray_init_host_ms` includes the one-time GPU wait when profiling.
The cache falls back to the reference calculation if it exceeds the driver's
maximum shader-storage buffer size; `ray_cache_MiB=0` identifies that case.

Build `test_viosim_gpu_rays` and pass the texture directory to compare cached and
reference pixels on a hardware EGL/OpenGL renderer. This explicit GPU test is
not included in the regular CTest suite, which also runs on machines without a GPU.

```bash
cmake --build /tmp/slam_build_viosim/standalone --parallel 10 --target test_viosim_gpu_rays
/tmp/slam_build_viosim/standalone/apps/test_viosim_gpu_rays assets/textures
```

Camera projection defaults to `camera_model: seucm`, using the SEUCM convention. Set `alpha` (default 4/9) and `beta` (default 1)
in the simulator YAML. The distortion center is centered: `eu = cx`, `ev = cy`.
These defaults preserve the rays of the previous `xi: 0.8` UCM projection.
For the original model, select `camera_model: ucm`; only that model uses `xi`.
CPU/GPU rendering, live calibration and exported calibration use the selected
model. SEUCM exports eight intrinsics: `[fx, fy, cx, cy, eu, ev, alpha, beta]`.

## Build

Required: CMake 3.16 or later, a C++17 compiler and libpng development files.
On Debian/Ubuntu, install `cmake`, `ninja-build`, `g++` and `libpng-dev`.
OpenMP and EGL/OpenGL development files are optional.

```bash
git clone git@github.com:bezout/vio_sim.git
cd vio_sim
cmake -S . -B /tmp/slam_build_viosim/standalone -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/slam_build_viosim/standalone --parallel 10
ctest --test-dir /tmp/slam_build_viosim/standalone --output-on-failure
```

| CMake option | Effect |
|---|---|
| `VIOSIM_BUILD_APPS` | Builds the generator and preview; defaults to ON for standalone builds, OFF as a subdirectory. |
| `VIOSIM_OPENMP` | Enables parallel rendering when OpenMP is available. |
| `VIOSIM_GPU` | Enables the EGL/OpenGL backend when development files are available. |
| `BUILD_TESTING` | Builds CPU tests and the explicit hardware GPU test. |
| `BUILD_SHARED_LIBS` | Selects shared libraries in standalone builds; defaults to OFF. |

For library-only use, configure with `-DVIOSIM_BUILD_APPS=OFF`.
The project can also be included with `add_subdirectory(vio_sim)`; consumers
link to `viosim_core` and use headers from `include/viosim`.

The build copies `assets/` next to the generator. Install with
`cmake --install /tmp/slam_build_viosim/standalone --prefix /tmp/viosim-install`.
Executables go to `bin`, libraries to `lib`, headers to `include`, and
textures/examples to `share/viosim`.

## Generate a sequence

```bash
<build>/apps/viosim generate \
  --scene office \
  --trajectory figure8 \
  --difficulty easy \
  --duration 8 \
  --period 8 \
  --threads 4 \
  --output /tmp/viosim_office_easy_8s
```

The included configurations are under `examples/`:

```bash
<build>/apps/viosim generate \
  --config examples/office_hard.yaml \
  --output /tmp/viosim_office_hard
```

The `*_live.yaml` examples use long finite durations for SLAM endurance runs;
the other examples remain short enough for offline dataset generation.

### Named presets

The following built-in presets are intended for quick live-SLAM experiments and
do not require a YAML file: `office-controller`, `office-bias-validation`,
`corridor-10m`, `warehouse-scanner`, and `lab-room-scanner`.
They all use `trajectory: random_long` and therefore the collision-aware,
continuously certified trajectory generator.
`corridor-10m` stays static for 2 s, travels 10 m along the corridor in 5 s,
then stays static for a further 2 s. Its IMU has neither noise nor initial
bias; the `easy` camera rendering remains unchanged.
`office-bias-validation` starts with 2 s static, then uses 28 s of 6-DoF
controller motion. It has ideal images and IMU measurements, so the V2 bias
reference is exactly zero on all six axes.

```bash
<build>/apps/viosim list-presets
<build>/apps/viosim generate \
  --preset office-bias-validation --output /tmp/viosim_bias_validation
```

CLI options override a preset, so `--preset warehouse-scanner --duration 300`
extends that live source to five minutes. The `random_long` presets use the
built-in motion-profile defaults; use YAML when a motion parameter itself needs
to change.

`--samples 1` renders one ray per output pixel; the default `--samples 2`
renders four. `--threads` parallelizes camera groups. Existing nonempty output
directories are refused and are never deleted by VioSim.

The supported scenes are `office`, `office_tag`, `office_tag_near`, `corridor`, `warehouse`, and `lab_room`. The trajectories
are `figure8`, `ellipse`, `back_and_forth`, `slow_forward`, `straight`, `pure_rotation`, and
`static`, plus the streaming `random_long` family. `straight` travels forward by `size` metres during `period` seconds,
after `warmup` seconds stationary and before at least `cooldown` seconds
stationary. Use `viosim --help`, `viosim list-scenes`, and
`viosim list-trajectories` for the complete option list.

For legacy trajectories, `yaw_turn_start`, `yaw_turn_duration`, and
`yaw_turn_deg` apply one smooth extra heading turn. The angle and angular
velocity enter both rendered camera poses and IMU ground truth. For example,
`yaw_offset_deg: 155`, `yaw_turn_start: 8`, `yaw_turn_duration: 3`, and
`yaw_turn_deg: -155` start facing away and reach the original heading at 11 s.

### Two-device AprilTag replay

`office_tag` adds one fixed `tag36h11`, ID 0, to the office's rear wall.
`office_tag_near` keeps that scene and tag, but moves the trajectory anchor
from `[0, 1.6, 1.25]` to `[-0.25, 3.0, 1.25]` m for a range comparison.
Visibility and background geometry change with the viewpoint; check both
devices' tag detection counts when comparing the variants.
Its center is `[-1.7, 7, 2]` m in the shared world, facing `-Y`.
The black-border side length used for pose estimation is **0.8 m**; the full
square including the white margin is 1 m. These tag properties are also saved
in `scene.json`. The marker uses the vendored AprilTag ID 0 pattern and the
normal scene renderer, including camera projection, occlusion, blur and noise.

### Long deterministic motion

`trajectory: random_long` selects the collision-aware streaming generator. Its
six modes are `rectilinear`, `circular`, `scanner`, `xr_headset`, `controller`,
and `mower`. Keep the recording duration and sensor rates in the dataset YAML;
put the motion limits and planning parameters in a dedicated file:

```yaml
scene: lab_room
trajectory: random_long
motion_mode: scanner
motion_seed: 1234
motion_config: motion/scanner.yaml
duration: 120
camera_hz: 50
imu_hz: 500
```

Relative `motion_config` paths are resolved from the dataset YAML directory.
The six complete profiles are under `examples/motion/`, and
`examples/lab_room_random_long_scanner.yaml` is ready for offline or live use.
The `examples/warehouse_random_long_*.yaml` files exercise every profile in the
warehouse scene.
`motion_seed` is a full-width unsigned integer and is independent of `seed`,
which continues to control image and IMU noise. The dataset-level
`motion_mode` and `motion_seed` override `mode` and `seed` from the motion file.

The motion file exposes start pose, kinematic bounds, primitive timing and
extent, collision margins, pauses, planner budgets, circular/controller and
mower parameters. Angles there are radians. Camera lever arms always come from
the current rig. `motion_reference_camera` selects the physical camera whose
center and optical axis define outward circular motion; it also orients the
initial IMU pose so that this camera looks forward during the static prefix.
Its `R_BC` calibration remains unchanged, so IMU data keeps the calibrated IMU
axes. `-1` uses a virtual center and the configured body optical axis.

The generator certifies complete primitives against the current room and every
scene box. It keeps bounded planning history, so its memory does not grow with
recording length. VioSim uses independent chronological cursors for camera and
IMU states; camera poses are prepared in bounded chunks before parallel image
rendering. Existing trajectories retain the meanings of `period`, `size`,
`warmup`, and `ramp`. For `random_long`, the host `warmup` initializes the
motion profile; an explicit `warmup` in `motion_config` overrides it. The fixed
distance convenience preset uses the host `warmup` unconditionally. Generated datasets record `motion_resolved.yaml`, the
exact accepted primitive sequence in `motion_primitives.csv`, the algorithm
version, the full motion seed, the scene snapshot and a trajectory fingerprint.

`lab_room` models a 7.6 × 8.8 × 3.15 m laboratory with lockers, cupboards,
rear shelving and a clear central aisle. Its optional `lab_reflections` switch
adds approximate view-dependent floor highlights; `lab_vignette` controls radial
attenuation from 0 to 0.95. These settings do not affect other scenes, geometry,
poses, IMU data or timestamps. The `lab_room_*.yaml` files provide ready-to-run
appearance and difficulty presets.

An inspection application renders three fixed stereo viewpoints through the
same CPU renderer used by dataset generation:

```bash
/tmp/slam_build_viosim/standalone/apps/viosim_lab_preview /tmp/viosim-lab-inspection
```

## Camera rigs

Without `camera_count`, VioSim preserves its original stereo rig configured by
`baseline`, `right_yaw_deg`, and `right_roll_deg`. An explicit rig contains
exactly two or four cameras:

```yaml
camera_count: 4
cam0_rotation_bc: [1, 0, 0, 0, 0, 1, 0, -1, 0]
cam0_translation_bc: [-0.04, 0, 0.03]
# ... cam1 through cam3 ...
```

Each rotation is a row-major `R_BC`; each translation is `t_BC` in metres.
Together they map camera coordinates into the IMU/body frame. Every camera must
provide both values, and rotations are checked for orthonormality and a
determinant of +1. See `examples/office_quad.yaml` for a complete four-camera
configuration.

## Output format

```text
sequence/
├── cam0/data.csv
├── cam0/data/<timestamp_ns>.png
├── cam1/data.csv
├── cam1/data/<timestamp_ns>.png
├── cam*/...                    # cam2 and cam3 for a four-camera rig
├── imu.csv
├── imu_clean.csv
├── groundtruth.csv
├── camera_poses.csv
├── calibration.yaml
├── config_used.yaml
├── motion_resolved.yaml       # random_long only
├── motion_primitives.csv      # random_long only
├── scene.json
├── manifest.json
└── debug/preview.png
```

Timestamps are integer nanoseconds. `imu.csv` stores
`timestamp_ns,wx,wy,wz,ax,ay,az,bgx,bgy,bgz,bax,bay,baz`; `imu_clean.csv` omits
the bias columns. Angular rates are rad/s, positions are metres, and the
accelerometer reports specific force in m/s². `T_BC` in `calibration.yaml` maps
camera coordinates into the IMU/body frame.

At the default 640×400 resolution, VioSim emits synchronized 50 Hz stereo images
and 500 Hz IMU data with a 0.08 m baseline, 120 degree horizontal FOV and SEUCM
parameters `alpha=4/9`, `beta=1`. An eight-second recording contains 400 camera groups and
4,001 IMU samples.



### VioSim benchmark presets and live ground truth

The additional `test-*` presets cover accuracy, fast motion, degraded images,
online recalibration, known IMU biases, moving initialization, camera blackout,
map budgets and two/four-camera throughput. `viosim list-benchmarks` lists descriptions; `viosim config --preset NAME`
prints the effective flat YAML without generating images.

New flat YAML / CLI options:

```yaml
initial_gyro_bias: [0.005, -0.003, 0.002] # rad/s; overrides difficulty, including ideal
initial_accel_bias: [0.08, -0.06, 0.04]  # m/s²; same rule
# White noise and bias random walk remain independently configurable.
gravity: 9.80665                       # m/s², historical default remains 9.81
motion_time_offset: 4                  # random_long: start inside the motion (seconds)
motion_time_scale: 1                   # random_long: v scales by s, a by s²
truth_output: /tmp/run-truth           # optional LIVE export; empty/new directory
reported_focal_scale: 1.03             # LIVE device calibration only; physical fx/fy unchanged
reported_baseline_scale: 1             # LIVE device translations only; physical rig unchanged
blackout_start: 12                     # camera images become black; IMU is unaffected
blackout_duration: 2                   # seconds; zero disables the blackout
```

`truth_output` is independent of `output`, which is the offline dataset path.
The live truth file uses the actual output timestamp and the unmodified
simulation timestamp for each IMU sample. It contains the exact bias realization,
not an independently regenerated random sequence. No images are stored.
The two `reported_*` options only affect the live device, not offline dataset
calibration. `motion_time_scale` also scales the initial static interval;
the primitive tape remains in generator time, with the offset/scale recorded
in `config_used.yaml`. The benchmark presets use 9.80665 m/s² to avoid confusing
a gravity-reference discrepancy with an estimated accelerometer bias.
