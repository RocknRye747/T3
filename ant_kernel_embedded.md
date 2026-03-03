# Ant Kernel Embedded Integration Notes (ESP32-S3 + iPhone ANE)

## System split

### Runs on ESP32-S3 directly
- Core ant state machine:
  - edge weight storage (compressed weighted grid)
  - evaporation and reinforcement update
  - weighted transition sampling
  - active Z vector list and decay
- Safety constraints:
  - enforce `epsilon > 0`
  - preserve connected graph topology
- Control output:
  - current target vertex for the reflex kernel

### Runs on iPhone ANE (over USB-C)
- Heavy perception inference pipeline:
  - WiFi CSI pre-processing
  - DensePose / human localization fusion
  - risk classifier producing `activity_class`, `niosh_risk`, `confidence`
- Optional multi-frame smoothing and outlier rejection
- Packetization of Z vectors for transport to ESP32-S3

## Z vector data path
1. WiFi CSI stream is processed in the phone-side pipeline.
2. Pipeline outputs structured Z vectors.
3. Z vectors are serialized and sent over USB-C (CDC/serial framing or custom bulk protocol).
4. ESP32 task parses Z vectors and calls `inject_z_vector(...)` equivalent logic.
5. On each navigation tick, kernel applies decaying exogenous penalties and emits next target node.

## Coupling to `reflex.S` motor control
- Ant kernel decides **WHERE** to go (next graph vertex / local waypoint).
- `reflex.S` decides **HOW** to move (motor timing, low-level actuation).
- Interface contract:
  - input to reflex: `(target_x, target_y)` or a heading + step primitive
  - feedback to ant kernel: movement completion / failure / slip status
- If reflex reports failure, ant kernel can suppress failed edge reinforcement to avoid biasing blocked routes.

## Memory budget guidance (ESP32-S3)
Target hardware budget: 512 KB SRAM + 8 MB PSRAM.

Suggested representation for a 10x10 grid:
- Nodes: 100
- Undirected edges: 180
- Weights as `float32`: 180 x 4 B = 720 B
- Optional fixed-point `uint16` with scaling reduces SRAM pressure further
- Visit counters + active Z vectors + queues: typically < 10 KB

For larger grids (e.g., 32x32):
- edges ~ 1984
- `float32` weights ~ 7.8 KB
Still practical in SRAM; history buffers can move to PSRAM if needed.

## Timing requirements
Recommended control cadence:
- Kernel tick: 20-50 Hz (every 20-50 ms)
- Z-vector update: 5-20 Hz (sensor dependent)

At each tick, work is O(E_local + degree(node)) if updates are sparse,
or O(E) if global evaporation is done eagerly. For embedded efficiency:
- use lazy evaporation bookkeeping (store global scale)
- apply exact per-edge clamp when an edge is touched

## FreeRTOS wrapping plan

### Task layout
- `task_perception_io` (medium priority)
  - receives/decodes Z vectors
  - pushes into lock-free queue
- `task_ant_kernel` (high priority, periodic)
  - drains Z queue
  - applies kernel step
  - publishes target vertex
- `task_reflex_bridge` (high priority)
  - converts target vertex to reflex commands
  - handles completion/fault feedback

### Synchronization
- Single-writer ownership of weight matrix in `task_ant_kernel`
- Message queues for Z vectors and motor feedback
- Ring buffer telemetry for debugging distribution convergence

## Practical deployment notes
- Keep `epsilon` compile-time configurable but strictly > 0.
- Validate connectivity once at boot and after any map edits.
- Seed RNG deterministically for reproducible bench tests; switch to hardware entropy in field mode.
- Include watchdog-aware timing so kernel ticks cannot starve reflex motor loop.
