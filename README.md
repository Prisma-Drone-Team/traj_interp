# Trajectory Interpolator (traj_interp)

Nodo ROS2 dedicato per l'interpolazione smooth di traiettorie per droni PX4.

## 🎯 Caratteristiche

- ✅ **Interpolazione ffilter**: Algoritmo dal lee_controller per traiettorie smooth
- ✅ **Seguimento continuo**: Waypoints seguiti in sequenza senza fermarsi  
- ✅ **Gestione path dinamica**: Nuovo path scarta quello precedente immediatamente
- ✅ **Compatibilità PX4**: Interface standard con px4_msgs
- ✅ **Fix path switching**: Risolto il bug di commutazione tra path
- ✅ **Auto-heading**: Yaw calcolato automaticamente dalla direzione di movimento
- ✅ **Offboard all'avvio**: Modalità offboard attiva subito (solo hover)
- ✅ **Auto-arm intelligente**: Arm automatico al primo path o se il drone è atterrato
- ✅ **Auto-disarm**: Disarm automatico quando atterrato

## 🚀 Quick Start

### Compilazione
```bash
cd /your/ros2_workspace
colcon build --packages-select traj_interp
source install/setup.bash
```

### Avvio
```bash
ros2 launch traj_interp trajectory_interpolator.launch.py
```

### Test con Path
```bash
# Quadrato
ros2 topic pub /trajectory_path nav_msgs/Path '{
  header: {frame_id: "odom"},
  poses: [
    {header: {frame_id: "odom"}, pose: {position: {x: 0.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}},
    {header: {frame_id: "odom"}, pose: {position: {x: 2.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}},
    {header: {frame_id: "odom"}, pose: {position: {x: 2.0, y: 2.0, z: 5.0}, orientation: {w: 1.0}}},
    {header: {frame_id: "odom"}, pose: {position: {x: 0.0, y: 2.0, z: 5.0}, orientation: {w: 1.0}}},
    {header: {frame_id: "odom"}, pose: {position: {x: 0.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}}
  ]
}' --once

# Linea
ros2 topic pub /trajectory_path nav_msgs/Path '{
  header: {frame_id: "odom"},
  poses: [
    {header: {frame_id: "odom"}, pose: {position: {x: 0.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}},
    {header: {frame_id: "odom"}, pose: {position: {x: 1.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}},
    {header: {frame_id: "odom"}, pose: {position: {x: 2.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}},
    {header: {frame_id: "odom"}, pose: {position: {x: 3.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}}
  ]
}' --once
```

## 📊 Topics

### Input
- `/trajectory_path` (nav_msgs/Path) - Waypoints da seguire
- `/px4/odometry/out` (nav_msgs/Odometry) - Posizione drone

### Output  
- `fmu/in/offboard_control_mode` (px4_msgs/OffboardControlMode)
- `fmu/in/vehicle_command` (px4_msgs/VehicleCommand)
- `/px4/trajectory_setpoint_enu` (trajectory_msgs/MultiDOFJointTrajectoryPoint)
- `/trajectory_interpolator/status` (std_msgs/String)

## ⚙️ Parametri Configurabili

File: `config/trajectory_interpolator.yaml`

```yaml
# Performance
ref_vel_max: 1.0          # Velocità massima [m/s]
ref_acc_max: 1.0          # Accelerazione massima [m/s²]
ref_jerk_max: 2.0         # Jerk massimo [m/s³]

# Smoothness  
ref_omega: 1.0            # Frequenza filtro [rad/s] 
ref_zeta: 0.7             # Smorzamento

# Precision
waypoint_tolerance: 0.1   # Tolleranza waypoint [m]
control_frequency: 50.0   # Frequenza controllo [Hz]
```

## 🔧 Stati del Nodo

- **IDLE**: In attesa di path
- **FOLLOWING_TRAJECTORY**: Seguendo traiettoria
- **STOPPED**: Fermato per nuovo path (transizione)

## 🐛 Monitoring

```bash
# Status nodo
ros2 topic echo /trajectory_interpolator/status

# Setpoint interpolati
ros2 topic echo /px4/trajectory_setpoint_enu

# Path ricevuto
ros2 topic echo /trajectory_path
```

## 🔄 Algoritmo ffilter

L'algoritmo implementa limitazioni fisiche per:
- **Jerk** (derivata accelerazione)
- **Accelerazione** 
- **Velocità**

Con filtro del secondo ordine:
```
acceleration = ω² × error - 2ζω × velocity
```

## ✅ Fix Implementati

- **Path switching**: Nuovo path ora riparte automaticamente
- **State machine**: Gestione corretta stati IDLE/STOPPED/FOLLOWING  
- **Thread safety**: Mutex per accesso coda waypoints
- **Smooth stopping**: Arresto graduale su nuovo path

---

