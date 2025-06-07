package Physics_Engine.LandingController;

import static Physics_Engine.LandingController.Constants.*; // Import all static members from the Constants class for easy access to physical constants and limits.

public class LanderController {
    /**
     * Control Gains - These are tuning parameters for the Proportional-Derivative (PD) controller.
     * They determine how strongly the lander reacts to errors in its position, velocity, and orientation.
     * These values typically need to be adjusted through simulation to achieve stable and efficient landing.
     */
    private final double proportionalGainVerticalPosition = 0.5; // Kp: Proportional gain for vertical (y-axis) position error.
    private final double derivativeGainVerticalVelocity = 1.0;   // Kd: Derivative gain for vertical (y-axis) velocity error, helps damp oscillations.
    private final double proportionalGainHorizontalPosition = 0.1; // Kp: Proportional gain for horizontal (x-axis) position error.
    private final double derivativeGainHorizontalVelocity = 0.5; // Kd: Derivative gain for horizontal (x-axis) velocity error.
    private final double proportionalGainOrientationAngle = 5.0; // Kp: Proportional gain for orientation angle (rotation around Z-axis) error.
    private final double derivativeGainAngularVelocity = 2.0;    // Kd: Derivative gain for angular velocity (rate of rotation) error, helps damp rotational oscillations.

    /**
     * Calculates the control inputs (thrust and torque) required for the lander
     * to achieve its desired state based on its current state.
     * This method implements a PD control strategy.
     *
     * @param currentLanderState The current state of the lander, including its position, velocity, orientation, and angular velocity.
     * @return A ControlInputs object containing the calculated thrust and torque.
     */
    public ControlInputs calculateControlInputs(LanderState currentLanderState) {
        // --- Vertical Control (Y-axis) ---
        // The goal is to bring the lander to a vertical position of 0 and vertical velocity of 0.
        // desired_ay = Kp_y * (desired_y - current_y) + Kd_y * (desired_vy - current_vy)
        double desiredVerticalAcceleration = proportionalGainVerticalPosition * (0 - currentLanderState.getPositionVector().getY()) +
                                             derivativeGainVerticalVelocity * (0 - currentLanderState.getVelocityVector().getY());
        
        // Calculate the raw thrust magnitude needed to achieve the desired vertical acceleration.
        // This calculation also accounts for the constant gravitational pull of Titan and the lander's current tilt (orientation).
        // thrust_raw = (desired_ay + gravity) / cos(orientation)
        double rawThrustMagnitude = (desiredVerticalAcceleration + GRAVITY_TITAN) / Math.cos(currentLanderState.getOrientation());

        // Clamp the raw thrust magnitude to ensure it stays within the physical limits of the lander's engine.
        // Thrust cannot be negative (pulling up) and has a maximum limit.
        double finalThrustMagnitude = clamp(rawThrustMagnitude, 0, THRUST_MAX);

        // --- Horizontal Control (X-axis) ---
        // The goal is to bring the lander to a horizontal position of 0 and horizontal velocity of 0.
        // desired_ax = Kp_x * (desired_x - current_x) + Kd_x * (desired_vx - current_vx)
        double desiredHorizontalAcceleration = proportionalGainHorizontalPosition * (0 - currentLanderState.getPositionVector().getX()) +
                                               derivativeGainHorizontalVelocity * (0 - currentLanderState.getVelocityVector().getX());
        
        // Calculate the desired orientation angle (tilt) of the lander.
        // This angle is determined by the desired horizontal acceleration and the current vertical thrust.
        // The lander tilts to direct a component of its thrust horizontally.
        double desiredOrientationAngleRadians = Math.atan2(desiredHorizontalAcceleration, (finalThrustMagnitude * Math.cos(currentLanderState.getOrientation())));

        // --- Orientation Control (Rotation) ---
        // Calculate the error in the lander's orientation angle.
        // The normalizeAngle helper ensures the error is always the shortest path between two angles (-PI to PI).
        double orientationErrorRadians = normalizeAngle(desiredOrientationAngleRadians - currentLanderState.getOrientation());
        
        // Calculate the torque (rotational force) needed to correct the lander's orientation.
        // torque = Kp_theta * (desired_theta - current_theta) + Kd_omega * (desired_omega - current_omega)
        // Desired angular velocity (omega) is typically 0 for stable landing.
        double appliedTorqueMagnitude = proportionalGainOrientationAngle * orientationErrorRadians +
                                        derivativeGainAngularVelocity * (0 - currentLanderState.getAngularVelocity());

        // Clamp the applied torque magnitude to ensure it stays within the physical limits of the lander's thrusters.
        appliedTorqueMagnitude = clamp(appliedTorqueMagnitude, -TORQUE_MAX, TORQUE_MAX);

        // Return the calculated thrust and torque as control inputs.
        return new ControlInputs(finalThrustMagnitude, appliedTorqueMagnitude);
    }

    /**
     * Helper method to clamp a given value within a specified minimum and maximum range.
     * This prevents control outputs from exceeding physical limits.
     *
     * @param valueToClamp The input value to be clamped.
     * @param minimumValue The lower bound of the clamping range.
     * @param maximumValue The upper bound of the clamping range.
     * @return The clamped value.
     */
    private double clamp(double valueToClamp, double minimumValue, double maximumValue) {
        return Math.max(minimumValue, Math.min(maximumValue, valueToClamp));
    }

    /**
     * Helper method to normalize an angle to the range (-PI, PI] radians.
     * This is important for angle calculations to ensure the shortest angular distance.
     * For example, an angle of 3PI becomes PI, and -3PI becomes -PI.
     *
     * @param angleInRadians The angle to be normalized, in radians.
     * @return The normalized angle in radians.
     */
    private double normalizeAngle(double angleInRadians) {
        return (angleInRadians + Math.PI) % (2 * Math.PI) - Math.PI;
    }
}