package Physics_Engine.LandingController;

import src.Physics_Engine.ODESolverRK4.RK4_ODESolver;
import src.Physics_Engine.GeneralComponents.Interfaces.SolarSystemInterface;
import src.Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import src.Physics_Engine.GeneralComponents.Vector;
import src.Physics_Engine.GeneralComponents.SolarSystem; // Assuming this is the concrete implementation of SolarSystemInterface

import java.util.ArrayList;

import static src.Physics_Engine.LandingController.Constants.*;

public class LanderSimulation {

    public static void main(String[] args) {
        // 1. Initialize LanderState with initial conditions
        // Example initial conditions (convert from km/s to m/s, km to m)
        double initialX = 1000.0; // 1 km
        double initialY = 5000.0; // 5 km
        double initialVx = -0.5 * KM_TO_M; // -0.5 km/s
        double initialVy = -0.1 * KM_TO_M; // -0.1 km/s
        double initialTheta = Math.PI / 4; // 45 degrees (pointing somewhat right)
        double initialOmega = 0.0;

        LanderObject initialLander = new LanderObject(
            new Vector(initialX, initialY, 0), // Assuming 2D for now, Z is 0
            new Vector(initialVx, initialVy, 0), // Assuming 2D for now, Z is 0
            LANDER_MASS, // Assuming LANDER_MASS is defined in Constants
            "Lander",
            initialTheta, initialOmega
        );

        // 2. Instantiate Controller, Wind Model, and ODE Function
        LanderController controller = new LanderController();
        WindModel windModel = new WindModel(0.05); // Max wind force 0.05 m/s^2
        LanderPhysicsFunction landerAccelerationFunction = new LanderPhysicsFunction(controller, windModel);
        LanderVelocityFunction landerVelocityFunction = new LanderVelocityFunction();

        // Setup SolarSystem for RK4_ODESolver
        SolarSystemInterface solarSystem = new SolarSystem();
        solarSystem.addSpaceObject(initialLander); // Add the lander to the system

        // Instantiate RK4_ODESolver
        RK4_ODESolver rk4Solver = new RK4_ODESolver();

        // 4. Simulation Loop
        double time = 0;
        double dt = 0.01; // Time step for simulation (tune this!)
        final double MAX_SIMULATION_TIME = 1000.0; // Max 1000 seconds to prevent infinite loops

        System.out.println("Starting Lander Simulation...");
        System.out.printf("Initial State: x=%.2f, y=%.2f, vx=%.2f, vy=%.2f, theta=%.2f, omega=%.2f%n",
            initialLander.getPositionVector().getX(), initialLander.getPositionVector().getY(),
            initialLander.getVelocityVector().getX(), initialLander.getVelocityVector().getY(),
            initialLander.getTheta(), initialLander.getOmega());

        LanderObject currentLander = initialLander;

        while (currentLander.getPositionVector().getY() > 0 && time < MAX_SIMULATION_TIME) {
            // Perform one step of integration for position and velocity using RK4_ODESolver
            // The RK4_ODESolver updates the SpaceObjects directly in the solarSystem list
            rk4Solver.ComputeODE(time, solarSystem, landerAccelerationFunction, landerVelocityFunction);

            // Manually perform RK4 step for theta and omega
            // k1
            ControlInputs inputs_k1 = controller.calculateControlInputs(currentLander);
            double dtheta_dt_k1 = currentLander.getOmega();
            double domega_dt_k1 = inputs_k1.torque;

            // Apply k1/2 to a temporary state for k2 calculation
            LanderObject tempLander_k2 = new LanderObject(
                currentLander.getPositionVector(),
                currentLander.getVelocityVector(),
                currentLander.getMass(),
                currentLander.getName(),
                currentLander.getTheta() + dtheta_dt_k1 * dt / 2,
                currentLander.getOmega() + domega_dt_k1 * dt / 2
            );

            // k2
            ControlInputs inputs_k2 = controller.calculateControlInputs(tempLander_k2);
            double dtheta_dt_k2 = tempLander_k2.getOmega();
            double domega_dt_k2 = inputs_k2.torque;

            // Apply k2/2 to a temporary state for k3 calculation
            LanderObject tempLander_k3 = new LanderObject(
                currentLander.getPositionVector(),
                currentLander.getVelocityVector(),
                currentLander.getMass(),
                currentLander.getName(),
                currentLander.getTheta() + dtheta_dt_k2 * dt / 2,
                currentLander.getOmega() + domega_dt_k2 * dt / 2
            );

            // k3
            ControlInputs inputs_k3 = controller.calculateControlInputs(tempLander_k3);
            double dtheta_dt_k3 = tempLander_k3.getOmega();
            double domega_dt_k3 = inputs_k3.torque;

            // Apply k3 to a temporary state for k4 calculation
            LanderObject tempLander_k4 = new LanderObject(
                currentLander.getPositionVector(),
                currentLander.getVelocityVector(),
                currentLander.getMass(),
                currentLander.getName(),
                currentLander.getTheta() + dtheta_dt_k3 * dt,
                currentLander.getOmega() + domega_dt_k3 * dt
            );

            // k4
            ControlInputs inputs_k4 = controller.calculateControlInputs(tempLander_k4);
            double dtheta_dt_k4 = tempLander_k4.getOmega();
            double domega_dt_k4 = inputs_k4.torque;

            // Update theta and omega using RK4 formula
            double newTheta = currentLander.getTheta() + (dt / 6.0) * (dtheta_dt_k1 + 2 * dtheta_dt_k2 + 2 * dtheta_dt_k3 + dtheta_dt_k4);
            double newOmega = currentLander.getOmega() + (dt / 6.0) * (domega_dt_k1 + 2 * domega_dt_k2 + 2 * domega_dt_k3 + domega_dt_k4);

            currentLander.setTheta(newTheta);
            currentLander.setOmega(newOmega);

            time += dt;

            // Optional: Print state periodically for debugging
            // if (time % 1.0 < dt) { // Print every second
            //     System.out.printf("Time: %.2f, State: x=%.2f, y=%.2f, vx=%.2f, vy=%.2f, theta=%.2f, omega=%.2f%n",
            //         time, currentLanderState.x, currentLanderState.y, currentLanderState.vx,
            //         currentLanderState.vy, currentLanderState.theta, currentLanderState.omega);
            // }
        }

        System.out.println("\nSimulation Ended.");
        System.out.printf("Final State: x=%.2f, y=%.2f, vx=%.2f, vy=%.2f, theta=%.2f, omega=%.2f%n",
            currentLander.getPositionVector().getX(), currentLander.getPositionVector().getY(),
            currentLander.getVelocityVector().getX(), currentLander.getVelocityVector().getY(),
            currentLander.getTheta(), currentLander.getOmega());

        // 5. Check success criteria
        boolean landedSuccessfully =
            Math.abs(currentLander.getPositionVector().getX()) <= DELTA_X &&
            Math.abs(normalizeAngle(currentLander.getTheta())) <= DELTA_THETA &&
            Math.abs(currentLander.getVelocityVector().getX()) <= EPSILON_X &&
            Math.abs(currentLander.getVelocityVector().getY()) <= EPSILON_Y &&
            Math.abs(currentLander.getOmega()) <= EPSILON_THETA;

        if (landedSuccessfully) {
            System.out.println("Lander successfully landed!");
        } else {
            System.out.println("Lander failed to land successfully.");
            if (currentLander.getPositionVector().getY() <= 0) {
                System.out.println("Reason: Hit ground but failed conditions.");
            } else if (time >= MAX_SIMULATION_TIME) {
                System.out.println("Reason: Simulation timed out before landing.");
            }
        }
    }

    // Helper method to normalize angle to (-PI, PI]
    private static double normalizeAngle(double angle) {
        return (angle + Math.PI) % (2 * Math.PI) - Math.PI;
    }
}