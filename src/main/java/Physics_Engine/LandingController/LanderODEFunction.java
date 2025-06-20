package Physics_Engine.LandingController;

import Physics_Engine.GeneralComponents.Vector;
import static Physics_Engine.LandingController.Constants.*;

public class LanderODEFunction  {
    private LanderController controller;
    private WindModel windModel;
    private Vector targetPosition;

    public LanderODEFunction(LanderController controller, WindModel windModel, Vector targetPosition) {
        this.controller = controller;
        this.windModel = windModel;
        this.targetPosition = targetPosition;
    }

    public void setTargetPosition(Vector targetPosition) {
        this.targetPosition = targetPosition;
    }



    /**
     * Computes the derivative of the state vector for the Lander.
     * The state vector is [x, y, vx, vy, theta, omega].
     *
     * @param state The current state vector [x, y, vx, vy, theta, omega].
     * @param time The current time.
     * @param params Optional parameters needed for the derivative computation.
     * @return The derivative of the state vector [dx/dt, dy/dt, dvx/dt, dvy/dt, dtheta/dt, domega/dt].
     */

    public double[] computeDerivative(double[] state, double time, double[] params) {
        // Convert raw state array to LanderState object for easier access and readability
        LanderState currentLanderState = new LanderState(
                state[0], state[1], state[2], state[3], state[4], state[5]
        );

        // Get control inputs (thrust and torque) from the controller based on the current state and target position
        ControlInputs inputs = controller.calculateControlInputs(currentLanderState, this.targetPosition);

        double altitude = currentLanderState.y; // Altitude is the y-component in this 2D simulation
        double airspeed = Math.sqrt(currentLanderState.vx * currentLanderState.vx + currentLanderState.vy * currentLanderState.vy); // Airspeed is the magnitude of the velocity

        double wind_force_x = windModel.getWindX(altitude, airspeed);
        double wind_force_y = windModel.getWindY(altitude, airspeed);

        // Calculate the derivatives of the state variables based on physics equations:
        // dx/dt = vx (horizontal velocity)
        double dx_dt = currentLanderState.vx;
        // dy/dt = vy (vertical velocity)
        double dy_dt = currentLanderState.vy;
        // dvx/dt = horizontal acceleration (thrust component + wind force)
        double dvx_dt = inputs.thrust * Math.sin(currentLanderState.theta) + wind_force_x;
        // dvy/dt = vertical acceleration (thrust component - gravity + wind force)
        double dvy_dt = inputs.thrust * Math.cos(currentLanderState.theta) - GRAVITY_TITAN + wind_force_y;
        // dtheta/dt = omega (angular velocity)
        double dtheta_dt = currentLanderState.omega;
        // domega/dt = angular acceleration (torque)
        double domega_dt = inputs.torque;

        // Return the array of derivatives
        return new double[]{dx_dt, dy_dt, dvx_dt, dvy_dt, dtheta_dt, domega_dt};
    }
}