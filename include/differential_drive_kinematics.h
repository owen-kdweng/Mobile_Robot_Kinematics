#ifndef DIFFERENTIAL_DRIVE_KINEMATICS_H
#define DIFFERENTIAL_DRIVE_KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>



/**
 * @brief Status codes returned by differential drive kinematics functions.
 */
typedef enum {
    DD_OK = 0,						/**< Operation successful */
    DD_ERROR = -1,					/**< General calculation error */
    DD_ERROR_INVALID_PARAMS = -2	/**< Invalid input parameters */
} dd_status_t;
// ------------------------------------------------------------------------



/**
 * @brief Physical parameters of a differential drive robot.
 *
 * All units are in SI:
 * - wheel_radius: meters (m)
 * - wheel_base:   meters (m)
 */
typedef struct {
    double wheel_radius;     // wheel radius (m)
    double wheel_base;       // distance between two wheels (m)
} dd_params_t;
// ------------------------------------------------------------------------


typedef struct {
    double x;
    double y;
    double z;
} vec3_t;

/**
 * @brief Robot body velocity (twist).
 *
 * Represents motion in the robot's local frame.
 *
 * - linear:  forward velocity (m/s)
 * - angular: rotational velocity around vertical axis (rad/s)
 */
typedef struct {
	vec3_t linear;			// m/s
	vec3_t angular;			// rad/s
}dd_twist_t;
// ------------------------------------------------------------------------



/**
 * @brief Angular velocities of the wheels.
 *
 * - left:  left wheel angular velocity (rad/s)
 * - right: right wheel angular velocity (rad/s)
 */
typedef struct {
	double left;			// rad/s
	double right;			// rad/s
}dd_wheel_speed_t;
// ------------------------------------------------------------------------



/**
 * @brief Compute forward kinematics for a differential drive robot.
 *
 * Converts wheel angular velocities into robot body velocity.
 *
 * Equations:
 *   v = (r / 2) * (wl + wr)
 *   w = (r / L) * (wr - wl)
 *
 * @param[in]  params        Pointer to robot physical parameters
 * @param[in]  wheel_speed   Pointer to wheel angular velocities
 * @param[out] out_twist     Output robot body velocity (twist)
 *
 * @return
 * - DD_OK on success
 * - DD_ERROR_INVALID_PARAMS if any input pointer is NULL or parameters are invalid
 * - DD_ERROR if numerical computation fails (NaN or Inf)
 */
dd_status_t dd_forward_kinematics(
	const dd_params_t *params,
	const dd_wheel_speed_t *wheel_speed,
	dd_twist_t *out_twist
);
// ------------------------------------------------------------------------



/**
 * @brief Compute inverse kinematics for a differential drive robot.
 *
 * Converts robot body velocity into wheel angular velocities.
 *
 * Equations:
 *   wl = (v - (L/2)*w) / r
 *   wr = (v + (L/2)*w) / r
 *
 * @param[in]  params            Pointer to robot physical parameters
 * @param[in]  twist             Pointer to robot body velocity
 * @param[out] out_wheel_speed   Output wheel angular velocities
 *
 * @return
 * - DD_OK on success
 * - DD_ERROR_INVALID_PARAMS if any input pointer is NULL or parameters are invalid
 * - DD_ERROR if numerical computation fails (NaN or Inf)
 */
dd_status_t  dd_inverse_kinematics(
	const dd_params_t *params,
	const dd_twist_t *twist,
	dd_wheel_speed_t *out_wheel_speed
);
// ------------------------------------------------------------------------



#ifdef __cplusplus
}
#endif

#endif
