# C Mobile Robot Kinematics Library

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
| `vec3_t`              | A set of three vectors                            |
| `dd_twist_t`          | Robot velocity (vec3_t linear, vec3_t angular)    |
| `dd_wheel_speed_t`    | Wheel angular velocities (left, right)            |
| `dd_status_t`         | Return status codes (OK / ERROR / INVALID PARAM)  |
---


##  Usage

### 1. Initialize Parameters
```bash
dd_params_t params;
params.wheel_radius = 0.05;   // 5 cm
params.wheel_base   = 0.30;   // 30 cm
```

### 2. Forward Kinematics
Convert wheel angular velocities → robot velocity (v, ω).
```c
dd_twist_t twist;
dd_wheel_speed_t wheel_speed;
wheel_speed.left  = 10.0;
wheel_speed.right = 12.0;

dd_forward_kinematics(&params, &ws, &twist);

if (status != DD_OK) {
    printf("Forward kinematics failed!\n");
    return -1;
}

printf("=== Forward Kinematics ===\n");
printf("Input wheel speeds:\n");
printf("  left  = %.3f rad/s\n", wheel_speed.left);
printf("  right = %.3f rad/s\n", wheel_speed.right);

printf("Output twist:\n");
printf("  linear.x  = %.6f m/s\n", twist.linear.x);
printf("  angular.z = %.6f rad/s\n\n", twist.angular.z);
```


### 3. Inverse Kinematics
```c
dd_wheel_speed_t wheel_speed;
dd_twist_t cmd = {0.0, 0.5};  // linear velocity, angular velocity

dd_inverse_kinematics(&params, &cmd, &wheel_speed);

if (status != DD_OK) {
    printf("Forward kinematics failed!\n");
    return -1;
}

printf("=== Inverse Kinematics ===\n");
printf("wheel speeds:\n");
printf("  left  = %.6f rad/s\n", wheel_speed.left);
printf("  right = %.6f rad/s\n\n", wheel_speed.right);
```


---

## License
MIT License