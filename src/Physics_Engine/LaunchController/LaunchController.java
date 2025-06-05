package src.Physics_Engine.LaunchController;

import src.Physics_Engine.LandingController.ControlInputs;
import src.Physics_Engine.LandingController.LanderState;
import static src.Physics_Engine.LandingController.Constants.*; // Import constants

public class LaunchController {

    private LanderState targetLanderState;

    // PID gains (tune these)
    private static final double KP_POS = 0.001; // Proportional gain for position error
    private static final double KD_VEL = 0.1;  // Derivative gain for velocity error
    private static final double KP_ANGLE = 0.5; // Proportional gain for angle error
    private static final double KD_ANGULAR_VELOCITY = 0.1; // Derivative gain for angular velocity error

    public LaunchController(LanderState targetLanderState) {
        this.targetLanderState = targetLanderState;
    }

    /**
     * Calculates the control inputs (thrust and gimbal angle) for the rocket during launch.
     *
     * @param currentState The current state of the lander.
     * @param deltaTime The time step.
     * @return The calculated control inputs.
     */
    public ControlInputs getControlInputs(LanderState currentState, double deltaTime) {
        // Calculate desired acceleration based on position and velocity errors
        double desiredAccX = KP_POS * (targetLanderState.x - currentState.x) + KD_VEL * (targetLanderState.vx - currentState.vx);
        double desiredAccY = KP_POS * (targetLanderState.y - currentState.y) + KD_VEL * (targetLanderState.vy - currentState.vy) + GRAVITY_TITAN;

        // Calculate desired thrust magnitude and angle
        double desiredThrustMagnitude = Math.sqrt(desiredAccX * desiredAccX + desiredAccY * desiredAccY) * SPACESHIP_MASS;
        double desiredThrustAngle = Math.atan2(desiredAccY, desiredAccX);

        // Clamp thrust to maximum
        double thrust = Math.min(desiredThrustMagnitude, THRUST_MAX);

        // Calculate gimbal angle relative to current orientation
        double gimbalAngle = normalizeAngle(desiredThrustAngle - currentState.theta);

        // Clamp gimbal angle change per time step based on TORQUE_MAX
        // TORQUE_MAX is max angular acceleration, so max angle change per dt is TORQUE_MAX * dt
        double maxGimbalChange = TORQUE_MAX * deltaTime;
        gimbalAngle = Math.max(-maxGimbalChange, Math.min(maxGimbalChange, gimbalAngle));

        return new ControlInputs(thrust, gimbalAngle);
    }

    // Helper method to normalize angle to (-PI, PI]
    private static double normalizeAngle(double angle) {
        return (angle + Math.PI) % (2 * Math.PI) - Math.PI;
    }
}