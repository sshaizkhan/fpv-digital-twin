# Project: Digital twin FPV simulator for my specific 5" quad (macOS, Apple Silicon)

## Goal
A flight simulator where the REAL Betaflight firmware (SITL target) flies a physics
model parameterized from measurements of my actual quad, controlled by my real radio,
viewed through an FPV camera. End state: the sim's gyro response to a given stick input
matches Blackbox logs from the real quad closely enough that rates, tune changes, and
later MSP/autonomy code can be tested in sim before touching hardware.

Non-goals: multiplayer, track editor, pretty graphics, flexible-frame dynamics.

## The real quad
- Frame: custom 3D-printed 5" X frame (heavier and less stiff than carbon)
- Stack: SpeedyBee F405 V4 FC + 55A BLHeli_S ESC (Bluejay), Betaflight [VERSION]
- Motors: EMAX ECO II 2807 1700KV x4
- Props: Gemfan Hurricane 51466 tri-blade
- Battery: 4S 1300mAh LiPo
- Radio: Jumper T-Pro V2 (EdgeTX), appears as USB HID joystick
- AUW: [GRAMS] g. CG offset: [X,Y,Z mm]
- Motor positions from CG (mm): [FL, FR, RL, RR]
- Inertia (kg m^2): Ixx=[ ], Iyy=[ ], Izz=[ ]  (if blank, estimate from geometry and flag it)
- Camera tilt: [DEG] deg, FOV: [DEG] deg
- Betaflight config: ./config/diff_all.txt

All physical parameters live in one file: ./config/quad.yaml. Every estimated (not
measured) value must carry `measured: false` so I know what to replace.

## Architecture
1. physics/  C++17, CMake, Eigen. Fixed-step 6DOF rigid body (quaternion attitude,
   RK4 or semi-implicit), 1 kHz minimum, decoupled from rendering.
   - Motor model: command -> first-order lag (time constant) -> RPM -> thrust = kT*w^2,
     torque = kQ*w^2, plus rotor inertia reaction torque on yaw.
   - Battery: OCV curve + internal resistance sag limiting available motor voltage.
   - Aero: linear + quadratic body drag, simple induced-drag term. Ground plane contact.
   - IMU synthesis: body rates + specific force, with configurable noise and bias.
   - Talks to Betaflight SITL over UDP using SITL's own packet structs (fdm packet in,
     motor/servo packet out). READ THE BETAFLIGHT SOURCE for the SITL target to get
     exact struct layouts, ports, units, axis conventions, and motor ordering. Do not
     guess these. Document them in docs/sitl_interface.md.
   - Reads the radio via SDL2 joystick API, maps axes/switches to RC channels
     (AETR + AUX arm/mode), sends to SITL's RC UDP input.
   - Publishes pose at 120 Hz over UDP to the viewer.
2. third_party/betaflight  git submodule pinned to the tag matching my FC firmware.
   Build the SITL target natively. If native macOS build fails after reasonable
   patching, provide a Dockerfile that builds and runs it with UDP/TCP ports mapped.
   Configurator must be able to connect over TCP so I can paste my diff.
3. viewer/  Godot 4 project. UDP pose listener, FPV camera with my tilt/FOV, simple
   ground + reference objects (gates, poles, grid) for depth cues, minimal OSD
   (throttle, battery voltage, armed state). Chase cam toggle for debugging.
4. tools/  Python. Blackbox log parser (orangebox), script to replay logged RC
   commands into the sim headless, overlay sim vs real gyro traces, and a
   least-squares fitter for kT scale, drag coefficients, and motor time constant.

## Phases and acceptance criteria
- Phase 0: repo scaffold, CMake build, quad.yaml schema, README with run steps.
- Phase 1: physics core standalone with unit tests: free fall = g, hover thrust
  balances weight, pure roll torque gives expected angular acceleration, energy sane.
- Phase 2: Betaflight SITL builds and runs; Configurator connects; physics<->SITL
  loop closed; quad arms and holds a stable hover with scripted RC input.
- Phase 3: radio input live; viewer running; I can fly acro from FPV view at a
  steady frame rate with end-to-end input latency measured and logged.
- Phase 4: tools pipeline: load a real Blackbox log, replay sticks, produce
  overlay plots and fitted parameters written back to quad.yaml.

## Working rules
- Verify every interface detail against source before coding against it. State
  what you verified and where.
- Coordinate frames: define one convention (document it), convert explicitly at
  the SITL boundary and at the Godot boundary. Most bugs will be here: wrong axis
  sign, wrong motor order, wrong spin direction. Write a test for each.
- Determinism: physics must run headless and faster than real time for replay.
- Small commits per milestone. Run tests before declaring a phase done.
- Ask me for any measured value you need instead of inventing one silently.

Start with Phase 0, then propose the Phase 1 plan before writing the physics.
