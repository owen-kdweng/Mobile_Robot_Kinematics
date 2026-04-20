# C Mobile Robot Kinematics Library (Differential Drive)

C language library for kinematics of wheeled robots.

This library is under development and currently supports the following bots.:

* Differential drive

---


## Project Structure
```
.
├── include/
│   └── differential_drive_kinematics.h
├── src/
│   └── differential_drive_kinematics.c
├── examples/
│   └── dd_example.c
├── CMakeLists.txt
├── README.md
└── LICENSE
```
---



## Build

```bash
mkdir build
cd build
cmake ..
make
./dd_example
```

---



## API Overview

| Function                  | Description                                                       |
| ------------------------- | ----------------------------------------------------------------- |
| `dd_forward_kinematics`   | Convert wheel angular velocities (wl, wr) → robot velocity (v, ω) |
| `dd_inverse_kinematics`   | Convert robot velocity (v, ω) → wheel angular velocities (wl, wr) |

---


## Data Structure Overview

| Function                  | Description                                   |
| --------------------- | ------------------------------------------------- |
| `dd_params_t`         | Robot physical parameters (wheel radius, base)    |
| `dd_twist_t`          | Robot velocity (linear v, angular ω)              |
| `dd_wheel_speed_t`    | Wheel angular velocities (left, right)            |
| `dd_status_t`         | Return status codes (OK / ERROR / INVALID PARAM)  |
---


##  Usage

### 1. Initialize Parameters
```bash
dd_params_t params = {
    0.05,   // wheel radius (meters)
    0.30    // wheel base (meters)
};
```

### 2. Forward Kinematics
Convert wheel angular velocities → robot velocity (v, ω).
```c
dd_twist_t twist;
dd_wheel_speed_t ws = {10.0, 10.0};  // left, right (rad/s)

dd_forward_kinematics(&params, &ws, &twist);
```

### 3. Inverse Kinematics
```c
dd_wheel_speed_t wheel_speed;
dd_twist_t cmd = {0.0, 0.5};  // linear velocity, angular velocity

dd_inverse_kinematics(&params, &cmd, &wheel_speed);
```


---

## License
MIT License