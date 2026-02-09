#ifndef MOTION_PROFILE_HPP
#define MOTION_PROFILE_HPP

#include <cmath>
#include <algorithm>

/**
 * Trapezoidal motion profile for feedforward velocity generation.
 *
 * Computes desired velocity based on remaining distance to goal:
 * - Far from goal: max velocity (cruise phase)
 * - Near goal: braking curve v = sqrt(2 * a * d)
 * - At goal: zero
 *
 * For omnidirectional robots: linear X/Y are combined into a single
 * 2D profile (shared magnitude), yaw is independent.
 */
class MotionProfile {
public:
    /**
     * Compute feedforward velocity for a single axis (e.g. yaw).
     *
     * @param distance  Signed distance to goal
     * @param max_vel   Maximum velocity (positive)
     * @param max_decel Maximum deceleration for braking (positive)
     * @return Signed velocity toward goal
     */
    static double compute_velocity(double distance, double max_vel, double max_decel) {
        double abs_dist = std::fabs(distance);

        if (abs_dist < 1e-6) {
            return 0.0;
        }

        // Braking curve: velocity needed to stop at goal with max deceleration
        double v_brake = std::sqrt(2.0 * max_decel * abs_dist);

        // Target velocity is minimum of max vel and braking vel
        double v_target = std::min(max_vel, v_brake);

        return std::copysign(v_target, distance);
    }

    /**
     * Compute feedforward velocities for 2D linear motion.
     * Uses total distance for profile magnitude, then splits into X/Y components.
     * This ensures the robot follows a straight-line path to the goal.
     *
     * @param error_x   X component of error (in base_frame)
     * @param error_y   Y component of error (in base_frame)
     * @param max_vel   Maximum linear velocity
     * @param max_decel Maximum linear deceleration
     * @param[out] vx   Output X velocity
     * @param[out] vy   Output Y velocity
     */
    static void compute_linear_velocity(double error_x, double error_y,
                                         double max_vel, double max_decel,
                                         double& vx, double& vy) {
        double distance = std::sqrt(error_x * error_x + error_y * error_y);

        if (distance < 1e-4) {
            vx = 0.0;
            vy = 0.0;
            return;
        }

        // Profile velocity magnitude based on total distance
        double v_brake = std::sqrt(2.0 * max_decel * distance);
        double v_magnitude = std::min(max_vel, v_brake);

        // Split into X/Y components (unit vector toward goal)
        vx = v_magnitude * (error_x / distance);
        vy = v_magnitude * (error_y / distance);
    }
};

#endif // MOTION_PROFILE_HPP
