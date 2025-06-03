package Physics_Engine.LaunchController;

// Import necessary classes (will need to be adjusted based on actual locations)
import Physics_Engine.LandingController.LanderState;
import Physics_Engine.LandingController.LanderObject;
import Physics_Engine.LandingController.LanderODEFunction;
import Physics_Engine.LandingController.ControlInputs;
import Physics_Engine.LandingController.WindModel;
import src.Physics_Engine.GeneralComponents.Vector;
import src.Physics_Engine.GeneralComponents.SolarSystem;
import src.Physics_Engine.GeneralComponents.Interfaces.SolarSystemInterface;
import src.Physics_Engine.ODESolverRK4.RK4_ODESolver;

import static Physics_Engine.LandingController.Constants.*; // Assuming Constants are needed

public class LaunchSimulation {

    public static void main(String[] args) {
        // Target state for the launch (original landing initial conditions)
        final double targetX = 1000.0; // 1 km
        final double targetY = 5000.0; // 5 km
        final double targetVx = -0.5 * KM_TO_M; // -0.5 km/s
        final double targetVy = -0.1 * KM_TO_M; // -0.1 km/s
        final double targetTheta = Math.PI / 4; // 45 degrees (pointing somewhat right)
        final double targetOmega = 0.0;

        // Initial conditions for launch (starting from a successful landing)
        double initialX = 0.0;
        double initialY = 0.0; // Start at ground level
        double initialVx = 0.0;
        double initialVy = 0.0; // Start with zero vertical velocity
        double initialTheta = Math.PI / 2; // Pointing straight up after landing
        double initialOmega = 0.0;

        // Initialize LanderState with initial conditions for launch
        LanderState currentLanderState = new LanderState(
            initialX, initialY, initialVx, initialVy, initialTheta, initialOmega
        );

        // Define target state
        LanderState targetLanderState = new LanderState(
            targetX, targetY, targetVx, targetVy, targetTheta, targetOmega
        );

        // Define tolerances for reaching the target state
        final double EPSILON_POSITION = 10.0; // meters
        final double EPSILON_VELOCITY = 0.1; // m/s
        final double EPSILON_ANGLE = Math.toRadians(1.0); // radians (1 degree)
        final double EPSILON_ANGULAR_VELOCITY = 0.01; // rad/s

        // Simulation Loop parameters
        double dt = 0.01; // Time step for simulation (tune this!)
        final double MAX_SIMULATION_TIME = 1000.0; // Max 1000 seconds to prevent infinite loops
        double time = 0; // Initialize time for the simulation timeout check

        // Instantiate Launcher with initial and target states
        Launcher launcher = new Launcher(currentLanderState, targetLanderState);

        System.out.println("Starting Rocket Launch Simulation...");
        System.out.printf("Initial State: x=%.2f, y=%.2f, vx=%.2f, vy=%.2f, theta=%.2f, omega=%.2f%n",
            currentLanderState.x, currentLanderState.y,
            currentLanderState.vx, currentLanderState.vy,
            currentLanderState.theta, currentLanderState.omega);

        // Simulate the launch
        LanderState finalState = launcher.simulateLaunch(dt);

        System.out.println("\nLaunch Simulation Completed.");
        System.out.printf("Final State from Launcher: x=%.2f, y=%.2f, vx=%.2f, vy=%.2f, theta=%.2f, omega=%.2f%n",
            finalState.x, finalState.y,
            finalState.vx, finalState.vy,
            finalState.theta, finalState.omega);

        // Check success criteria (if target state was reached)
        boolean targetStateReached =
            Math.abs(finalState.x - targetLanderState.x) <= EPSILON_POSITION &&
            Math.abs(finalState.y - targetLanderState.y) <= EPSILON_POSITION &&
            Math.abs(finalState.vx - targetLanderState.vx) <= EPSILON_VELOCITY &&
            Math.abs(finalState.vy - targetLanderState.vy) <= EPSILON_VELOCITY &&
            Math.abs(normalizeAngle(finalState.theta - targetLanderState.theta)) <= EPSILON_ANGLE &&
            Math.abs(finalState.omega - targetLanderState.omega) <= EPSILON_ANGULAR_VELOCITY;

        if (targetStateReached) {
            System.out.println("Rocket successfully reached target state!");
        } else {
            System.out.println("Rocket failed to reach target state.");
            if (launcher.isSimulationTimedOut()) {
                System.out.println("Reason: Simulation timed out before reaching target state.");
            } else {
                System.out.println("Reason: Simulation ended without reaching target state within tolerances.");
            }
        }
    }

    // Helper method to normalize angle to (-PI, PI]
    private static double normalizeAngle(double angle) {
        return (angle + Math.PI) % (2 * Math.PI) - Math.PI;
    }
}