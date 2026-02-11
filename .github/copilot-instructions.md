# Tello ROS2 Autonomous Drone Workspace

## Architecture Overview

ROS2 workspace for **autonomous Tello drone swarms** with ArUco marker detection, MiDaS depth estimation, and optional UWB-based positioning.

### Package Structure

| Package | Language | Purpose |
|---------|----------|---------|
| `tello_ros` | C++ | Tello SDK driver (UDP, image streaming) |
| `mission_control` | Python | ArUco-based state machine navigation |
| `mission_control_uwb` | Python | UWB-based state machine variant |
| `tello_vision` | C++/Python | ArUco detection + MiDaS depth |
| `tello_swarm` | C++ | Swarm coordination servers |
| `tello_bringup` | Launch | Configuration + launch files |

### Data Flow

```
Tello Drone → tello_driver → image_raw
                            ↓
        ┌───────────────────┴────────────────────┐
        ↓                                        ↓
   MiDaS (depth)                          ArUco (markers)
        ↓                                        ↓
   depth/analysis                      aruco_detection
        └───────────────────┬────────────────────┘
                            ↓
                    mission_control
                            ↓
                      cmd_vel/tello_action
```

## State Machine Architecture

The mission controller uses the **State Pattern** with a centralized `MissionContext` to avoid inter-state coupling.

### Key Design Principles

1. **States are stateless**: Use `__slots__`, access shared data via `self.context`
2. **Return-based transitions**: `execute() -> Optional[MissionState]` returns next state or `None` to stay
3. **Thin orchestrator**: `mission_control_node.py` delegates to specialized modules

### State Flow

`IDLE` → `TAKING_OFF` → `ASCENDING` → `STANDBY` → `[WAYPOINT_CENTERING → WAYPOINT_APPROACHING → WAYPOINT_ACTION]` → `SEARCHING` → `CENTERING` → `APPROACHING` → `CAMERA_SWITCHING` → `PRECISION_LANDING` → `LANDING` → `COMPLETING_MISSION` → `RESETTING`

### Adding a New State

```python
# 1. Create src/tello_nav/mission_control/mission_control/state_machine/logic/my_state.py
class MyState(BaseState):
    def execute(self) -> Optional[MissionState]:
        # Return MissionState.NEXT for transition, None to stay
        return MissionState.SEARCHING
```

2. Add enum to `states.py`
3. Register in `MissionManager._create_states()`
4. Export in `logic/__init__.py`

## Swarm Coordination

C++ servers in `tello_swarm` manage global resource conflicts:

| Server | Service | Purpose |
|--------|---------|---------|
| `marker_manager` | `/reserve_marker` | ArUco marker reservations (VICTIM: 1-8, FIRE: 9-14) |
| `waypoint_server` | `/reserve_waypoint` | Waypoint index reservations |
| `takeoff_server` | `/swarm_takeoff` | Synchronized multi-drone takeoff |
| `landing_server` | `/swarm_land` | Coordinated landing |

**Heartbeat pattern**: Reservations expire after ~6-8s without `/marker_heartbeat` or `/waypoint_heartbeat` renewal.

Launch all servers: `ros2 launch tello_bringup swarm_servers_launch.py`

## Critical Workflows

### Building

```bash
cd ~/tello_ros_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Multi-Drone Launch

```bash
# Terminal 1: Swarm coordination servers
ros2 launch tello_bringup swarm_servers_launch.py

# Terminal 2: All drones + perception
ros2 launch tello_bringup multi_drone_launch.py

# Terminal 3+: Mission control per drone
ros2 launch tello_bringup mission_control_launch.py drone_ns:=tello1
```

### Configuration Files

- Drone IPs/ports: `src/tello_bringup/config/drone_params.yaml`
- Waypoint sequences: `src/tello_bringup/missions/*.yaml` (format: `"id:50,action:ccw 90"`)
- UWB navigation: `src/tello_bringup/config/drone_params_uwb.yaml`

### Debugging

```bash
ros2 topic echo /tello1/mission_state      # Current state
ros2 topic echo /unavailable_markers       # Reserved markers
ros2 topic echo /unavailable_waypoints     # Reserved waypoints
ros2 topic echo /marker_registry           # Full marker status
```

## Project Conventions

### ROS2 Namespacing

- **Per-drone**: `/tello1/cmd_vel`, `/tello1/image_raw`, `/tello1/aruco_detection`
- **Global (swarm)**: `/reserve_marker`, `/reserve_waypoint`, `/unavailable_markers`
- **Transient local QoS**: Used for `/unavailable_*` topics so late subscribers get last state

### Waypoint Configuration Format

```yaml
waypoint_sequence: ["id:50,action:ccw 90", "id:51,action:none", "id:none,action:forward 50"]
```
- `id:none` = action-only waypoint (skip navigation)
- `action:none` = no post-arrival action

### Obstacle Avoidance (MiDaS)

`DepthMapAnalysis` divides view into 9 regions. Detection logic:
```python
if depth.middle_center.red > depth.middle_center.blue:
    # Obstacle ahead - rotate or evade
```

### Tello SDK Quirks

- `rc` commands don't return responses; use velocity control with care
- Auto-lands after 15s inactivity—keep sending commands
- Camera switch: `downvision` SDK command (Tello EDU only)

## UWB-Based Navigation (`mission_control_uwb`)

Alternative to ArUco-only navigation. Uses UWB (Ultra-Wideband) positioning for waypoint-to-waypoint flight, then ArUco for final marker search/landing.

### UWB Architecture

Three cooperating modules in `mission_control_uwb/uwb/`:

| Module | Responsibility |
|--------|---------------|
| `UWBNavigator` | Non-blocking position controller. Two strategies: `DRIVE` (rotate-then-forward) and `SLIDE` (holonomic strafe). Call `update()` each tick → returns `NavStatus.RUNNING/SUCCESS/BLOCKED` |
| `WaypointManager` | Loads waypoints from JSON file, tracks sequence index, provides `advance()`/`skip_current()`/`is_complete()` |
| `WaypointCoordinator` | Async reservation via `/reserve_waypoint` service, heartbeat publishing, unavailable-waypoint tracking. Falls back to uncoordinated mode if server unavailable |

### Key Differences from ArUco Variant

- **Single navigation state**: `WAYPOINT_NAVIGATION` replaces `WAYPOINT_CENTERING → WAYPOINT_APPROACHING → WAYPOINT_ACTION`
- **BaseState has extra slots**: `uwb_navigator`, `waypoint_coordinator` alongside `waypoint_manager`
- **MissionContext adds**: `waypoint_nav_start_time` for timeout tracking
- **Waypoints are JSON** (x/y coordinates), not YAML marker-id strings
- **State flow**: `STANDBY` → `WAYPOINT_NAVIGATION` → `SEARCHING` → (ArUco landing) → `LANDING`

### UWB Waypoint JSON Format

```json
{
  "waypoints": [
    {"x": 0.7, "y": 0.7, "label": "start"},
    {"x": 3.5, "y": 5.0, "label": "checkpoint_2"}
  ]
}
```
Files go in `src/tello_bringup/missions/`. Relative filenames in params are resolved to this directory.

### UWB Launch

```bash
# Per-drone mission params in missions/mission_waypoints_uwb.yaml
ros2 launch tello_bringup mission_control_uwb_launch.py drone_ns:=tello7
```

Per-drone config (`missions/mission_waypoints_uwb.yaml`) sets `waypoints_file`, `initial_search_height`, and `standby_delay` (staggered takeoff timing).

Drone-to-UWB-tag mapping is in `config/drone_params_uwb.yaml` (`uwb_tag_id` field).

## Key Files

| Purpose | File |
|---------|------|
| State base class | `src/tello_nav/mission_control/mission_control/state_machine/base_state.py` |
| State definitions | `src/tello_nav/mission_control/mission_control/state_machine/states.py` |
| Shared context | `src/tello_nav/mission_control/mission_control/state_machine/mission_context.py` |
| Marker handler | `src/tello_nav/mission_control/mission_control/aruco/aruco_marker_handler.py` |
| Waypoint manager (ArUco) | `src/tello_nav/mission_control/mission_control/aruco/waypoint_manager.py` |
| Swarm marker server | `src/tello_swarm/tello_swarm/src/marker_manager.cpp` |
| Swarm waypoint server | `src/tello_swarm/tello_swarm/src/waypoint_server.cpp` |
| UWB navigator | `src/tello_nav/mission_control/mission_control_uwb/uwb/uwb_navigator.py` |
| UWB waypoint manager | `src/tello_nav/mission_control/mission_control_uwb/uwb/waypoint_manager.py` |
| UWB waypoint coordinator | `src/tello_nav/mission_control/mission_control_uwb/uwb/waypoint_coordinator.py` |
| UWB navigation state | `src/tello_nav/mission_control/mission_control_uwb/state_machine/logic/waypoint_navigation.py` |
| UWB orchestrator node | `src/tello_nav/mission_control/mission_control_uwb/mission_control_uwb_node.py` |
| UWB mission config | `src/tello_bringup/missions/mission_waypoints_uwb.yaml` |
