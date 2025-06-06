package src.main.java.Physics_Engine.LandingController;

import static src.main.java.Physics_Engine.LandingController.Constants.*;

import src.main.java.Physics_Engine.GeneralComponents.Interfaces.SolarSystemInterface;


import src.main.java.Physics_Engine.GeneralComponents.SolarSystem;
import src.main.java.Physics_Engine.GeneralComponents.Vector;
import src.main.java.Physics_Engine.ODESolverRK4.RK4_ODESolver; // Assuming this is the concrete implementation of SolarSystemInterface

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

        // Initialize LanderState with initial conditions
        LanderState currentLanderState = new LanderState(
            initialX, initialY, initialVx, initialVy, initialTheta, initialOmega
        );

        // Create a LanderObject for the SolarSystem based on the initial state
        LanderObject landerObjectInSolarSystem = new LanderObject(
            new Vector(currentLanderState.x, currentLanderState.y, 0), // Assuming 2D for now, Z is 0
            new Vector(currentLanderState.vx, currentLanderState.vy, 0), // Assuming 2D for now, Z is 0
            SPACESHIP_MASS, // Assuming SPACESHIP_MASS is defined in Constants
            "Lander",
            currentLanderState.theta, currentLanderState.omega
        );

        // 2. Instantiate Controller, Wind Model, and ODE Function
        LanderController controller = new LanderController();
        WindModel windModel = new WindModel(0.05); // Max wind force 0.05 m/s^2
        // Instantiate the unified LanderODEFunction
        LanderODEFunction landerODEFunction = new LanderODEFunction(controller, windModel);

        // Setup SolarSystem for RK4_ODESolver
        SolarSystemInterface solarSystem = new SolarSystem();
        solarSystem.getSolarSystem().add(landerObjectInSolarSystem); // Add the lander object to the system

        // Instantiate RK4_ODESolver
        RK4_ODESolver rk4Solver = new RK4_ODESolver();

        // 4. Simulation Loop
        double time = 0;
        double dt = 0.01; // Time step for simulation (tune this!)
        final double MAX_SIMULATION_TIME = 1000.0; // Max 1000 seconds to prevent infinite loops

        System.out.println("Starting Lander Simulation...");
        System.out.printf("Initial State: x=%.2f, y=%.2f, vx=%.2f, vy=%.2f, theta=%.2f, omega=%.2f%n",
            currentLanderState.x, currentLanderState.y,
            currentLanderState.vx, currentLanderState.vy,
            currentLanderState.theta, currentLanderState.omega);

        // Simulation Loop
        while (currentLanderState.y > 0 && time < MAX_SIMULATION_TIME) {
            // Get the current state as a double array
            double[] currentStateArray = {
                currentLanderState.x, currentLanderState.y,
                currentLanderState.vx, currentLanderState.vy,
                currentLanderState.theta, currentLanderState.omega
            };

            // Perform one step of integration for the full state using the new RK4_ODESolver method
            double[] nextStateArray = rk4Solver.computeODE(currentStateArray, time, dt, landerODEFunction, null); // Pass null for params if not needed

            // Update the LanderState object with the new state
            currentLanderState.x = nextStateArray[0];
            currentLanderState.y = nextStateArray[1];
            currentLanderState.vx = nextStateArray[2];
            currentLanderState.vy = nextStateArray[3];
            currentLanderState.theta = nextStateArray[4];
            currentLanderState.omega = nextStateArray[5];

            // Update the LanderObject in the SolarSystem to match the LanderState
            landerObjectInSolarSystem.setPosition(new Vector(currentLanderState.x, currentLanderState.y, 0)); // Assuming 2D
            landerObjectInSolarSystem.setVelocity(new Vector(currentLanderState.vx, currentLanderState.vy, 0)); // Assuming 2D
            landerObjectInSolarSystem.setOrientation(currentLanderState.theta);
            landerObjectInSolarSystem.setAngularVelocity(currentLanderState.omega);


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
            currentLanderState.x, currentLanderState.y,
            currentLanderState.vx, currentLanderState.vy,
            currentLanderState.theta, currentLanderState.omega);

        // 5. Check success criteria
        boolean landedSuccessfully =
            Math.abs(currentLanderState.x) <= DELTA_X &&
            Math.abs(normalizeAngle(currentLanderState.theta)) <= DELTA_THETA &&
            Math.abs(currentLanderState.vx) <= EPSILON_X &&
            Math.abs(currentLanderState.vy) <= EPSILON_Y &&
            Math.abs(currentLanderState.omega) <= EPSILON_THETA;

        if (landedSuccessfully) {
            System.out.println("Lander successfully landed!");
        } else {
            System.out.println("Lander failed to land successfully.");
            if (currentLanderState.y <= 0) {
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