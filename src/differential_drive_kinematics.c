#include <math.h>
#include "differential_drive_kinematics.h"


dd_status_t dd_forward_kinematics(
    const dd_params_t *params,
    const dd_wheel_speed_t *wheel_speed,
    dd_twist_t *out_twist
)
{
    if (params == NULL || wheel_speed == NULL || out_twist == NULL) {
        return DD_ERROR_INVALID_PARAMS;
    }

    if (params->wheel_radius <= 0.0 || params->wheel_base <= 0.0) {
        return DD_ERROR_INVALID_PARAMS;
    }

    const double r  = params->wheel_radius;
    const double L  = params->wheel_base;
    const double wl = wheel_speed->left;
    const double wr = wheel_speed->right;

    const double linear  = 0.5 * r * (wl + wr);
    const double angular = (r / L) * (wr - wl);

    if (!isfinite(linear) || !isfinite(angular)) {
        return DD_ERROR;
    }

    out_twist->linear.x = linear;
    out_twist->angular.z = angular;

    return DD_OK;
}



dd_status_t dd_inverse_kinematics(
    const dd_params_t *params,
    const dd_twist_t *twist,
    dd_wheel_speed_t *out_wheel_speed
)
{
    if (params == NULL || twist == NULL || out_wheel_speed == NULL) {
        return DD_ERROR_INVALID_PARAMS;
    }

    if (params->wheel_radius <= 0.0 || params->wheel_base <= 0.0) {
        return DD_ERROR_INVALID_PARAMS;
    }

    const double r = params->wheel_radius;
    const double L = params->wheel_base;
    const double v = twist->linear.x;
    const double w = twist->angular.z;

    const double left  = (v - 0.5 * L * w) / r;
    const double right = (v + 0.5 * L * w) / r;

    if (!isfinite(left) || !isfinite(right)) {
        return DD_ERROR;
    }

    out_wheel_speed->left = left;
    out_wheel_speed->right = right;

    return DD_OK;
}





