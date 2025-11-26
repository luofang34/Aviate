# Aviate

Aviate is a minimal, deterministic, hard-real-time motion control kernel responsible only for state estimation and stabilization control — not navigation, communication, mission management, or human-machine interface.


## 2. Design Philosophy

### 2.1 Aviate Does Exactly Three Things

1. **State Estimation** — Attitude, position, velocity, angular rate estimation
2. **Stabilization & Control** — Rate loop, attitude loop, velocity/altitude/position hold
3. **Actuation Output** — Force/torque commands mapped to actuator outputs via mixer

### 2.2 Aviate Never Does
- ❌ Navigation (waypoints, procedures, LNAV/VNAV)
- ❌ Mission systems / autopilot management
- ❌ Maps / charts / databases
- ❌ Networking (TCP/UDP/WiFi/LTE)
- ❌ File systems / logging
- ❌ UI / GCS / cloud platforms
- ❌ Operating system dependencies
