#include <stdio.h>
#include <stdlib.h>

#include "differential_drive_kinematics.h"


int main(int argc, char *argv[]) {
	
	dd_twist_t twist;
	dd_wheel_speed_t wheel_speed;

	dd_params_t params = {0.05, 0.30};   // r=5cm, L=30cm
	dd_wheel_speed_t ws = {10.0, 10.0};
    dd_forward_kinematics(&params, &ws, &twist);

    printf("Forward:\n");
    printf("linear  = %.3f m/s\n", twist.linear);
    printf("angular = %.3f rad/s\n", twist.angular);

    dd_twist_t cmd = {0.0, 0.5};
    dd_inverse_kinematics(&params, &cmd, &wheel_speed);

    printf("Inverse:\n");
    printf("left  = %.3f rad/s\n", wheel_speed.left);
    printf("right = %.3f rad/s\n", wheel_speed.right);
	
	return 0;
}
