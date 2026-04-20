#include <stdio.h>
#include "differential_drive_kinematics.h"

int main(void)
{
    // -------------------------------
    // 1. Setting robot parameters
    // -------------------------------
    dd_params_t params;
    params.wheel_radius = 0.05;   // 5 cm
    params.wheel_base   = 0.30;   // 30 cm


    // -------------------------------
    // 2. Assuming wheel speed (rad/s)
    // -------------------------------
    dd_wheel_speed_t wheel_speed;
    wheel_speed.left  = 10.0;
    wheel_speed.right = 12.0;


    // -------------------------------
    // 3. Forward Kinematics
    // -------------------------------
    dd_twist_t twist;

    dd_status_t status = dd_forward_kinematics(
        &params,
        &wheel_speed,
        &twist
    );

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


    // -------------------------------
    // 4. Inverse Kinematics
    // -------------------------------
    status = dd_inverse_kinematics(
        &params,
        &twist,
        &wheel_speed
    );

    if (status != DD_OK) {
        printf("Inverse kinematics failed!\n");
        return -1;
    }

    printf("=== Inverse Kinematics ===\n");
    printf("wheel speeds:\n");
    printf("  left  = %.6f rad/s\n", wheel_speed.left);
    printf("  right = %.6f rad/s\n\n", wheel_speed.right);


    return 0;
}