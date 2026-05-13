# RA6A — 3D-Printed 6-Axis Robotic Arm

📺 **Demo:** https://youtu.be/l62hkhJTfqE?si=eneqJugdLhpxHiAf

Design, construction, and control of a 6-axis robotic arm built around an STM32F439ZI microcontroller and ROS 2 Humble with MoveIt 2. Capable of pick-and-place operation with vision-based cube detection.

Contact me if you have any questions, I'd love to see what the communtiy does with this.

---

## ⚠️ Important Warnings — Read Before Building

### CAD files are incomplete and contain errors

The STEP files in `hardware/cad/` are a **work-in-progress reference**, not a finished build package. If you intend to replicate this arm, be aware of the following:

- **Idler pulleys are not modeled** — every belt-driven joint requires idler pulleys for proper belt tensioning and routing. These are **mandatory** for the arm to function. You must add them yourself based on your motor placement and belt length.
- **Joint 2 (shoulder) deflects under load** — the J2 mechanical design is structurally weak and visibly deflects when the arm extends or carries payload. The cantilever geometry concentrates stress at the joint, and PLA+ creep makes it worse over time. A replicator should reinforce J2 with metal brackets, redesign the joint with a thicker cross-section, or use a co-axial drive configuration instead of the offset design used here. This is a known unresolved issue and one of the biggest weaknesses of the current design.
- **Belts are not modeled** — HTD-5M timing belts are used throughout. Lengths must be measured from your physical build.
- **Many small errors exist** — incorrect tolerances on some bearing pockets, hole misalignments, missing fillets, and a few non-printable overhangs. Expect to fix issues as you print.
- **The gripper is not my design** — it's adapted from the AR4 open-source arm by Chris Annin (Annin Robotics). The original AR4 gripper files are available at https://www.anninrobotics.com — see the Acknowledgements section.

If you're replicating this design, treat the CAD as a starting point — you will need to do your own engineering review and modifications.

### Hardcoded configuration to change

Before running anything, update these values for your setup:

- **iPhone camera IP** in the vision scripts (default `192.168.100.206` — your network will assign a different one)
- **Serial port** for the STM32 (default `/dev/ttyACM0` on Linux)
- **Camera calibration file path** in `scripts/aruco_detect.py` and `scripts/cube_detector_node.py`
- **ArUco marker positions** in `MARKER_POSITIONS_BASE` — these are measured from *my* workspace; you must re-measure them for yours (see the calibration section below)
- **Table height** — `TABLE_Z` is hardcoded to `0.273` m (the height of my table surface above the robot's `base_link` origin). You must measure your own table height and update this value in `scripts/aruco_detect.py` and `scripts/cube_detector_node.py`

---

## Hardware Overview

| Component | Part | Notes |
|---|---|---|
| MCU | STM32F439ZI Nucleo-144 | Bare-metal firmware |
| J1–J3 motors | NEMA 34 stepper | High-torque base joints |
| J1–J3 drivers | CL86T closed-loop | 60 V PSU each |
| J4–J6 motors | NEMA 23 stepper | Wrist joints |
| J4–J6 drivers | CL57T / CL57Y closed-loop | 36 V PSU each |
| Gripper | FT5330M servo | **Adapted from AR4** — software PWM on PC11 |
| Transmission | HTD-5M timing belts | Idler pulleys required (not in CAD) |
| Logic level | 3.3 V → 5 V shifter | Between STM32 and driver inputs |
| Structure | PLA+, 3D printed | Creality K2 Plus |

**Manual homing buttons:** LEFT (PC8), RIGHT (PC9), CONFIRM (PC10).

---

## Software Architecture

```
┌─────────────────┐    plan trajectory    ┌────────────────────┐
│   MoveIt 2      │ ─────────────────────▶│  ra6a_hardware     │
│   (planning)    │                       │  (ROS 2 control)   │
└─────────────────┘                       └─────────┬──────────┘
                                                    │ 60 Hz serial
                                                    │ rosserial topics
                                                    ▼
                                          ┌─────────────────────┐
                                          │  STM32F439ZI        │
                                          │  - trapezoidal exec │
                                          │  - software PWM     │
                                          │  - manual homing    │
                                          └─────────────────────┘
```

MoveIt does all trajectory planning. The `ra6a_hardware` package forwards waypoints to the STM32 at 60 Hz over serial. The STM32 executes waypoints with trapezoidal velocity profiling and reports state back at 20 Hz.

**rosserial topic IDs:**
- `101` — joint command
- `102` — joint state feedback (20 Hz)
- `103` — homing complete
- `104` — motion complete
- `105` — gripper servo

---

## Prerequisites

- Ubuntu 22.04 with ROS 2 Humble
- MoveIt 2 (`sudo apt install ros-humble-moveit`)
- `pick_ik` plugin (`sudo apt install ros-humble-pick-ik`)
- Python 3 with `opencv-python`, `numpy`, `pyserial`, `pyyaml`
- For firmware build: GCC ARM toolchain (`arm-none-eabi-gcc`), `make`
- DroidCam app on iPhone for vision pipeline (free version is enough)

---

## Setup — Step by Step

### 1. Clone the repo

```bash
git clone https://github.com/Prospector-E/RA6A.git
cd RA6A
```

### 2. Build and flash STM32 firmware

The firmware is built with a Makefile — **no STM32CubeIDE required**.

```bash
cd firmware
make clean
make
```

This produces `build/ra6a.bin`. To flash, plug the Nucleo board into USB and copy the binary onto its mounted drive:

```bash
# Linux
cp build/ra6a.bin /media/$USER/NODE_F439ZI/

# Windows (in Git Bash)
cp build/ra6a.bin /x/      # replace /x/ with the Nucleo's drive letter
```

The board reflashes itself and resets automatically.

### 3. Build the ROS 2 workspace

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Add the source line to your `.bashrc` to avoid repeating it every terminal:
```bash
echo "source ~/RA6A/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 4. Update the serial port (if needed)

The STM32 typically appears as `/dev/ttyACM0`. Check with:
```bash
ls /dev/ttyACM*
```
If yours is different, update the `serial_port` parameter in `ros2_ws/src/ra6a_hardware/config/ra6a_hardware.yaml`.

Give your user permission to use serial without sudo:
```bash
sudo usermod -aG dialout $USER
# Log out and back in for it to take effect
```

---

## Running the Arm

### 1. Manual homing (always first after power-up)

When the STM32 boots, all joints are at position zero in software but at arbitrary positions in reality. Home each joint manually:

1. Power on the arm.
2. Hold the **LEFT (PC8)** or **RIGHT (PC9)** button — the currently active joint moves in that direction.
3. Position the joint at its mechanical home reference.
4. Press **CONFIRM (PC10)** to save that position and advance to the next joint.
5. Repeat for all 6 joints.

Homed positions are saved to STM32 flash, so subsequent power-ups remember them. You only need to re-home if the arm is moved by hand or the flash is erased.

### 2. Launch MoveIt + hardware interface

```bash
ros2 launch ra6a_moveit_config servo.launch.py
```

This brings up MoveIt with the Servo plugin, RViz, and connects to the STM32 over serial. You should see the arm pose in RViz match the real arm.

### 3. Run pick-and-place

In a new terminal:
```bash
cd ~/RA6A/scripts
python3 pick_and_place_vision.py
```

The script runs the sequence: REST → PICK → grip → REST → PLACE → release → REST.

Edit the pick/place coordinates at the top of the script for your workspace.

---

## Running the Vision Pipeline

The vision pipeline involves three calibration steps, done in this order:

1. **Camera intrinsics** — characterize the iPhone camera (lens distortion, focal length). One-time per camera.
2. **ArUco marker calibration** — tell the code where the table markers physically sit in the robot's frame. Required for the camera-to-robot transform.
3. **Vision-to-arm calibration** — correct for residual systematic errors between detected cube positions and where the arm actually reaches. Optional but strongly recommended.

### 1. Set up DroidCam on iPhone

1. Install **DroidCam** from the App Store.
2. Connect the iPhone to the same WiFi as your Linux machine.
3. Open the app — note the IP address shown (e.g. `192.168.100.206`).
4. In settings, set resolution to 720p and lock exposure by tapping and holding on the workspace.

### 2. Update the IP in the scripts

The IP is hardcoded in three files (your iPhone's IP will be different):

```bash
grep -rn "192.168" scripts/
```

Edit each file and change `IPHONE_IP = '192.168.100.206'` to your iPhone's IP.

### 3. Camera intrinsics calibration (`calibrate_camera.py`)

This step measures the iPhone's lens distortion and focal length using a printed checkerboard. The output is `camera_calibration.yaml`, which the detection scripts load. Done once per camera (or whenever you change cameras).

1. Print a checkerboard pattern on A4 paper. The defaults in the script are **9×6 inner corners** with **26 mm squares** — measure your printed squares and update `CHECKERBOARD` and `SQUARE_SIZE_M` constants if yours differ.
2. Tape it flat to a rigid board (so it doesn't curl).
3. Run the script:
   ```bash
   python3 scripts/calibrate_camera.py
   ```
4. A live feed window opens. Hold the board in front of the iPhone — when the script detects the corners, they're outlined in green.
5. Press **SPACE** to capture a frame. Capture 15–20 frames from different angles, distances, and positions (corners of frame, edges, close, far, tilted).
6. Press **Q** to compute and save `camera_calibration.yaml`.
7. Update `CALIB_FILE` paths in `aruco_detect.py` and `cube_detector_node.py` to point to wherever you saved it.

### 4. ArUco marker calibration (manual — must be done for your workspace)

The vision pipeline locates the iPhone camera in the robot's frame by detecting four printed ArUco markers placed at known positions on the table. **You must physically measure each marker's position relative to the robot's `base_link` and update the code.** There is no automated procedure — this is a manual measurement step.

**Robot frame convention (`base_link`):**
- Origin is at the center of the robot's base, on the bottom mounting surface
- **+X axis** points to the back of the robot (away from the front face)
- **+Y axis** points to the right (when standing behind the robot)
- **+Z axis** points up

**Setup procedure:**

1. **Print four ArUco markers**, **50 mm** each, with IDs **0, 1, 2, 3**. Use the same dictionary the code expects — check the `ARUCO_DICT` constant in `aruco_detect.py` (default `DICT_ARUCO_ORIGINAL`).

2. **Stick them flat on your table surface** at the corners of the workspace. They must be:
   - **Flat** (no curling or tilt — back them with cardboard or stick to a rigid surface)
   - **Visible to the iPhone camera** at all times
   - **Spread out** (the further apart, the more accurate the camera pose solve)

3. **Measure the table height.** With a ruler or caliper, measure from the bottom of the robot's base plate (the `base_link` origin Z = 0 plane) up to the top of the table surface. Record this value in **meters**. Example: 27.3 cm above the base → `TABLE_Z = 0.273`.

4. **Measure each marker's center position** in the robot frame:
   - Measure X (back/forward) and Y (right/left) of each marker's center from the robot base origin.
   - Z is the same as `TABLE_Z` for all four (they're all on the table surface).
   - Use **meters with signs**. Markers in front of the robot have negative X. Markers to the left of the robot have negative Y.
   - Tip: stick a piece of tape on the table aligned with the robot's center, run another tape strip out from the base for the X axis, and measure offsets from there.

5. **Update the constants** in both `scripts/aruco_detect.py` and `scripts/cube_detector_node.py`:

   ```python
   TABLE_Z = 0.273   # ← your measured table height (meters)

   MARKER_POSITIONS_BASE = {
       3: np.array([-0.370,  0.164, TABLE_Z]),   # ← your measured X, Y for marker 3
       2: np.array([-0.370, -0.268, TABLE_Z]),   # ← marker 2
       1: np.array([-0.190,  0.164, TABLE_Z]),   # ← marker 1
       0: np.array([-0.190, -0.268, TABLE_Z]),   # ← marker 0
   }
   ```

6. **Verify visually.** Run `python3 scripts/aruco_detect.py` — you should see the markers detected (with axes drawn on them) and any colored cubes on the table reporting `(x, y)` coordinates in `base_link`. Place a cube at a measured spot on the table and check that the script's reported position matches your ruler. If they disagree, your marker positions or `TABLE_Z` are off — re-measure.

**Why this matters:** a 5 mm measurement error on a marker translates into noticeable end-effector offsets during pick attempts. Don't eyeball it. Use a hard ruler or caliper and double-check signs.

### 5. Vision-to-arm calibration (`calibrate_vision.py`)

Even after camera intrinsics and ArUco calibration are done correctly, the position the camera reports for a cube and the position the arm actually reaches may still differ by a few millimeters or a small rotation, due to camera mount imperfection, base-frame misalignment, or small ArUco measurement errors. This script computes a 2D affine transform from "detected" to "real" coordinates and saves it to `~/vision_calibration.json`. The `pick_and_place_vision.py` script automatically loads and applies this transform.

**Strongly recommended.** Skipping this step usually results in pick attempts missing the cube by a few centimeters.

**Procedure:**

1. Place **three cubes** of different colors anywhere in the workspace (spread them out — corners of the workspace work best).

2. Start the supporting nodes in two separate terminals:
   ```bash
   # Terminal 1
   ros2 launch ra6a_moveit_config servo.launch.py

   # Terminal 2
   python3 scripts/cube_detector_node.py
   ```

3. In a third terminal, run the calibration script:
   ```bash
   python3 scripts/calibrate_vision.py
   ```

4. For each of the three cubes, the script prompts you to position the gripper:
   - Use RViz's interactive marker (plan + execute) to move the gripper **directly above the cube center**
   - Gripper should point **straight down**
   - Tip of the gripper jaws should be roughly **10 cm above the cube top**
   - When the gripper is in position, press **SPACE** in the calibration terminal to record the pair (detected position, actual position)

5. After three pairs are recorded, the script computes the transform and saves `~/vision_calibration.json`. Done.

6. **Re-run this calibration whenever:**
   - The iPhone is repositioned or remounted
   - The robot is moved relative to the table
   - The ArUco markers are repositioned
   - Pick attempts start consistently missing in a systematic direction

### 6. Run vision-based pick-and-place

```bash
# Terminal 1: MoveIt + hardware
ros2 launch ra6a_moveit_config servo.launch.py

# Terminal 2: cube detector
python3 scripts/cube_detector_node.py

# Terminal 3: pick-and-place driver
python3 scripts/pick_and_place_vision.py
```

---

## Repository Structure

```
RA6A/
├── README.md                   You are here
├── firmware/                   STM32 bare-metal C
│   ├── Core/                   Application code
│   ├── Drivers/                STM32 HAL
│   ├── Makefile                Build with `make`
│   └── *.ld                    Linker scripts
├── ros2_ws/src/
│   ├── ra6a_hardware/          ros2_control hardware interface
│   ├── ra6a_moveit_config/     MoveIt config + launch files
│   └── ra6a_description/       URDF, meshes, SRDF
├── scripts/                    Python control + vision
│   ├── pick_and_place_vision.py
│   ├── aruco_detect.py
│   ├── cube_detector_node.py
│   ├── calibrate_camera.py     Checkerboard intrinsics
│   └── calibrate_vision.py     3-cube affine transform
├── hardware/
│   ├── cad/                    STEP files (⚠️ incomplete — see warnings)
│   ├── stl/                    STL exports
│   └── schematics/             Wiring diagrams
└── docs/
    └── images/                 Photos and diagrams
```

---

## Known Issues and Limitations

- **CAD is incomplete** — see warnings at the top of this README.
- **J2 structural deflection** — the shoulder joint deflects under load and is the weakest link in the arm. The joint design should be reinforced or redesigned for higher-accuracy work. This single issue limits the overall achievable precision more than anything else.
- **Open-loop steppers** — no joint encoders, so position drift can occur if the arm hits hard limits. Re-home if motion becomes inaccurate.
- **Real-time teleop is jittery** — the current architecture is optimized for planned trajectories, not continuous streaming. Pick-and-place works smoothly; live teleop does not.
- **Singularities near vertical** — if the target pose is directly above the base, joint 5 reaches a singularity. Pre-rotate joint 1 to approach from an angle.
- **WiFi-dependent camera** — vision pipeline relies on DroidCam over WiFi. Network drops will pause detection.
- **ArUco calibration is manual** — there is no automated camera-to-robot calibration. Marker positions and table height must be physically measured and hardcoded into the scripts.
- **Vision calibration must be re-run** whenever the camera, robot, or markers are moved relative to each other.

---

## Acknowledgements

This project builds on existing open-source work. In particular:

- **Gripper design** — adapted from the **AR4** open-source 6-axis robotic arm by **Chris Annin** (Annin Robotics). The original mechanical design and the dual position-and-velocity interface concept were influential in this project's architecture. Source: https://www.anninrobotics.com and https://github.com/ycheng517/ar4_ros_driver
- **MoveIt 2** — the entire motion planning stack
- **ROS 2 Humble** and the `ros2_control` framework

If you replicate the gripper, please credit Chris Annin / Annin Robotics in your own documentation.

---

## License

Released under the MIT License — see `LICENSE` for details. CAD files and STLs are provided as-is with no warranty of fitness for any purpose (see warnings above). The gripper design is adapted from AR4 (Annin Robotics) — see Acknowledgements; please honor the original project's licensing terms when redistributing gripper-related files.
