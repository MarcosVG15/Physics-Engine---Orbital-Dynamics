package Physics_Engine.LaunchController;

// Import necessary classes (will need to be adjusted based on actual locations)
import static Physics_Engine.LandingController.Constants.*;
import Physics_Engine.LandingController.LanderODEFunction;
import Physics_Engine.LandingController.LanderObject;
import Physics_Engine.LandingController.LanderState;
import Physics_Engine.LandingController.WindModel;
import src.Physics_Engine.GeneralComponents.Interfaces.SolarSystemInterface;
import src.Physics_Engine.GeneralComponents.SolarSystem;
import src.Physics_Engine.GeneralComponents.Vector;
import src.Physics_Engine.ODESolverRK4.RK4_ODESolver;

public class Launcher {

    private LanderState initialLanderState;
    private LanderState targetLanderState;
    private LaunchController controller;
    private WindModel windModel;
    private LanderODEFunction landerODEFunction;
    private RK4_ODESolver rk4Solver;
    private LanderObject landerObjectInSolarSystem;

    // Define tolerances for reaching the target state
    private final double EPSILON_POSITION = 10.0; // meters
    private final double EPSILON_VELOCITY = 0.1; // m/s
    private final double EPSILON_ANGLE = Math.toRadians(1.0); // radians (1 degree)
    private final double EPSILON_ANGULAR_VELOCITY = 0.01; // rad/s
    private final double MAX_SIMULATION_TIME = 1000.0; // Max 1000 seconds
    private boolean simulationTimedOut = false;

    public Launcher(LanderState initialLanderState, LanderState targetLanderState) {
        this.initialLanderState = initialLanderState;
        this.targetLanderState = targetLanderState;

        // Instantiate Launch Controller, Wind Model, and ODE Function
        this.controller = new LaunchController(targetLanderState); // Use the new LaunchController with target state
        this.windModel = new WindModel(0.05); // Max wind force 0.05 m/s^2
        // Instantiate the unified LanderODEFunction (using the LaunchController)
        this.landerODEFunction = new LanderODEFunction(this.controller, this.windModel);

        // Create a LanderObject for the SolarSystem based on the initial state
        this.landerObjectInSolarSystem = new LanderObject(
            new Vector(initialLanderState.x, initialLanderState.y, 0), // Assuming 2D for now, Z is 0
            new Vector(initialLanderState.vx, initialLanderState.vy, 0), // Assuming 2D for now, Z is 0
            SPACESHIP_MASS, // Assuming SPACESHIP_MASS is defined in Constants
            "Lander",
            initialLanderState.theta, initialLanderState.omega
        );

        // Setup SolarSystem for RK4_ODESolver
        SolarSystemInterface solarSystem = new SolarSystem();
        solarSystem.getSolarSystem().add(this.landerObjectInSolarSystem); // Add the lander object to the system

        // Instantiate RK4_ODESolver
        this.rk4Solver = new RK4_ODESolver();
    }

    public LanderState simulateLaunch(double dt) {
        LanderState currentLanderState = new LanderState(initialLanderState.x, initialLanderState.y, initialLanderState.vx, initialLanderState.vy, initialLanderState.theta, initialLanderState.omega);
        double time = 0;

        System.out.println("Starting Rocket Launch Simulation...");
        System.out.printf("Initial State: x=%.2f, y=%.2f, vx=%.2f, vy=%.2f, theta=%.2f, omega=%.2f%n",
            currentLanderState.x, currentLanderState.y,
            currentLanderState.vx, currentLanderState.vy,
            currentLanderState.theta, currentLanderState.omega);

        // Simulation Loop
        // Continue as long as the target state is not reached and within max simulation time
        while (time < MAX_SIMULATION_TIME &&
               (Math.abs(currentLanderState.x - targetLanderState.x) > EPSILON_POSITION ||
                Math.abs(currentLanderState.y - targetLanderState.y) > EPSILON_POSITION ||
                Math.abs(currentLanderState.vx - targetLanderState.vx) > EPSILON_VELOCITY ||
                Math.abs(currentLanderState.vy - targetLanderState.vy) > EPSILON_VELOCITY ||
                Math.abs(normalizeAngle(currentLanderState.theta - targetLanderState.theta)) > EPSILON_ANGLE ||
                Math.abs(currentLanderState.omega - targetLanderState.omega) > EPSILON_ANGULAR_VELOCITY)) {

            // Get the current state as a double array
            double[] currentStateArray = {
                currentLanderState.x, currentLanderState.y,
                currentLanderState.vx, currentLanderState.vy,
                currentLanderState.theta, currentLanderState.omega
            };

            // Perform one step of integration for the full state using the RK4_ODESolver
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

        // Check success criteria (if target state was reached)
        boolean targetStateReached =
            Math.abs(currentLanderState.x - targetLanderState.x) <= EPSILON_POSITION &&
            Math.abs(currentLanderState.y - targetLanderState.y) <= EPSILON_POSITION &&
            Math.abs(currentLanderState.vx - targetLanderState.vx) <= EPSILON_VELOCITY &&
            Math.abs(currentLanderState.vy - targetLanderState.vy) <= EPSILON_VELOCITY &&
            Math.abs(normalizeAngle(currentLanderState.theta - targetLanderState.theta)) <= EPSILON_ANGLE &&
            Math.abs(currentLanderState.omega - targetLanderState.omega) <= EPSILON_ANGULAR_VELOCITY;

        if (targetStateReached) {
            System.out.println("Rocket successfully reached target state!");
        } else {
            System.out.println("Rocket failed to reach target state.");
            if (time >= MAX_SIMULATION_TIME) {
                System.out.println("Reason: Simulation timed out before reaching target state.");
            } else {
                System.out.println("Reason: Simulation ended without reaching target state within tolerances.");
            }
        }
        return currentLanderState;
    }

    public boolean isSimulationTimedOut() {
        return simulationTimedOut;
    }

    // Helper method to normalize angle to (-PI, PI]
    private static double normalizeAngle(double angle) {
        return (angle + Math.PI) % (2 * Math.PI) - Math.PI;
    }
}