package Physics_Engine.LandingController;

import static Physics_Engine.LandingController.Constants.*;

import Physics_Engine.GeneralComponents.Interfaces.SolarSystemInterface;
import Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import Physics_Engine.GeneralComponents.Interfaces.vectorInterface;
import Physics_Engine.GeneralComponents.SolarSystem;
import Physics_Engine.GeneralComponents.Vector;
import Physics_Engine.ODESolverRK4.RK4_ODESolver;
import Physics_Engine.LandingController.LandingPad;
import Physics_Engine.LandingController.TitanDrydenWindModel; // Add this import

public class LanderSimulation {

    public static void main(String[] args) {
        double dt = 0.01;
        final double MAX_SIMULATION_TIME = 365 * 24 * 60 * 60; // Simulate for one year in seconds
        double time = 0;

        // Setup SolarSystem for initial simulation phase
        SolarSystem solarSystem = new SolarSystem();
        LandingPad landingPad = new LandingPad(solarSystem);

        // Find the SpaceShip object
        SpaceObject spaceShip = null;
        for (SpaceObject object : solarSystem.getSolarSystem()) {
            if ("SpaceShip".equals(object.getName())) { // Assuming "SpaceShip" is the name set
                spaceShip = object;
                break;
            }
        }
        if (spaceShip == null) {
            // Handle case where SpaceShip is not found
            System.err.println("SpaceShip not found in SolarSystem!");
            return; // Exit simulation if SpaceShip is not found
        }

        System.out.println("Starting initial simulation phase to reach Z-threshold...");

        // Phase 1: Simulate until Z-threshold is met
        while (time < MAX_SIMULATION_TIME) {
            landingPad.updatePosition(); // Update the landing pad's position


            // Get the Z-values of SpaceShip and LandingPad
            double spaceShipZ = spaceShip.getPositionVector().getZ();
            double landingPadZ = landingPad.getPositionVector().getZ();

            // Check if the Z-values are within the threshold
            if (Math.abs(spaceShipZ - landingPadZ) <= Constants.Z_THRESHOLD) {
                System.out.println("Z-threshold reached. Transitioning to landing simulation.");
                break; // Exit Phase 1 loop
            }

            time += dt;
        }

        // Check if Z-threshold was reached within simulation time
        if (time >= MAX_SIMULATION_TIME) {
            System.out.println("Max simulation time reached before Z-threshold. Landing simulation aborted.");
            return; // Exit if threshold not reached
        }

        // Calculate relative position and velocity at the point the Z-threshold was met
        double relativeX = spaceShip.getPositionVector().getX() - landingPad.getPositionVector().getX();
        double relativeY = spaceShip.getPositionVector().getY() - landingPad.getPositionVector().getY();
        double relativeVx = spaceShip.getVelocityVector().getX() - landingPad.getVelocityVector().getX();
        double relativeVy = spaceShip.getVelocityVector().getY() - landingPad.getVelocityVector().getY();



        double kmToM = Constants.KM_TO_M;

        // Initialize LanderState with the calculated relative values
        LanderState currentLanderState = new LanderState(
                relativeX * kmToM, // Convert relative X from km to meters
                relativeY * kmToM, // Convert relative Y from km to meters
                relativeVx * kmToM, // Convert relative Vx from km/s to m/s
                relativeVy * kmToM, // Convert relative Vy from km/s to m/s
                0.0, // Initial orientation (will be set based on relative Vx next)
                0.0  // Initial angular velocity
        );

        // Set landerstates orientation based on relative X velocity at threshold
        if (relativeVx > 0) {
            currentLanderState.theta = 0; // Gaining in LandingPad-relative X, set orientation to 0 rads
        } else if (relativeVx < 0) {
            currentLanderState.theta = Math.PI; // Losing in relative X, set orientation to Pi rads
        }


        // Create a LanderObject for the SolarSystem based on the initial state of the landing phase
        // Note: This LanderObject might represents the lander in 3D

        LanderObject landerObjectInSolarSystem = new LanderObject(
                new Vector(currentLanderState.x, currentLanderState.y, 0), // Assuming 2D landing in XY plane
                new Vector(currentLanderState.vx, currentLanderState.vy, 0), // Assuming 2D landing in XY plane
                SPACESHIP_MASS, // Assuming SPACESHIP_MASS is defined in Constants
                "Lander",
                currentLanderState.theta, currentLanderState.omega
        );

        // 2. Instantiate Controller, Wind Model, and ODE Function for the landing phase
        LanderController controller = new LanderController();
        // Calculate initial airspeed for the wind model based on the relative velocity magnitude
        double initialAirspeed = Math.sqrt(currentLanderState.vx * currentLanderState.vx + currentLanderState.vy * currentLanderState.vy);
        WindModel windModel = new TitanDrydenWindModel(dt, initialAirspeed, currentLanderState.y); // Pass dt, initial airspeed, initial altitude
        LanderODEFunction landerODEFunction = new LanderODEFunction(controller, windModel, (Vector) landingPad.getPositionVector());

        // Instantiate RK4_ODESolver for the landing phase
        RK4_ODESolver rk4Solver = new RK4_ODESolver();

        // 4. Landing Simulation Loop (Phase 2)
        final double MAX_LANDING_TIME = 1000.0; // Max time for the landing phase
        double landingTime = 0; // Time specifically for the landing phase

        System.out.println("Starting Landing Simulation...");
        System.out.printf("Initial Landing State: x=%.2f, y=%.2f, vx=%.2f, vy=%.2f, theta=%.2f, omega=%.2f%n",
                currentLanderState.x, currentLanderState.y,
                currentLanderState.vx, currentLanderState.vy,
                currentLanderState.theta, currentLanderState.omega);


        // Landing Simulation Loop
        while (currentLanderState.y > 0 && landingTime < MAX_LANDING_TIME) {
            // In the landing phase, the lander's state is already relative to the landing pad
            // and in the landing pad's local coordinate system.
            // We no longer need to update landingPad position or transform between global/local
            // within this loop, assuming the landing pad is the reference frame.

            // Get the current state as a double array
            double[] currentStateArray = {
                    currentLanderState.x, currentLanderState.y,
                    currentLanderState.vx, currentLanderState.vy,
                    currentLanderState.theta, currentLanderState.omega
            };

            // Perform one step of integration using the LanderODEFunction
            double[] nextStateArray = rk4Solver.computeODE(currentStateArray, landingTime, dt, landerODEFunction, null);

            // Update the LanderState object with the new state
            currentLanderState.x = nextStateArray[0];
            currentLanderState.y = nextStateArray[1];
            currentLanderState.vx = nextStateArray[2];
            currentLanderState.vy = nextStateArray[3];
            currentLanderState.theta = nextStateArray[4];
            currentLanderState.omega = nextStateArray[5];

            landingTime += dt;

        }

        System.out.println("\nLanding Simulation Ended.");
        System.out.printf("Final Landing State: x=%.2f, y=%.2f, vx=%.2f, vy=%.2f, theta=%.2f, omega=%.2f%n",
                currentLanderState.x, currentLanderState.y,
                currentLanderState.vx, currentLanderState.vy,
                currentLanderState.theta, currentLanderState.omega);

        // 5. Check success criteria based on the final LanderState (which is already relative)
        boolean landedSuccessfully =
                Math.abs(currentLanderState.x) <= DELTA_X && // Check relative X
                        Math.abs(currentLanderState.y) <= DELTA_Y_LANDING_PAD && // Check relative Y (height above pad)
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
            } else if (landingTime >= MAX_LANDING_TIME) {
                System.out.println("Reason: Landing simulation timed out before landing.");
            }
        }
    }

    // Helper method to normalize angle to (-PI, PI]
    private static double normalizeAngle(double angle) {
        return (angle + Math.PI) % (2 * Math.PI) - Math.PI;
    }

    private static LanderState transformGlobalToLocal(LanderObject lander, LandingPad pad) {
        // 1. Translate: Lander's position relative to pad's position
        Vector relativeGlobalPos = (Vector) lander.getPositionVector().subtract(pad.getPositionVector());
        Vector relativeGlobalVel = (Vector) lander.getVelocityVector().subtract(pad.getVelocityVector());

        // 2. Rotate: Align with pad's local coordinate system
        // Pad's local X-axis (normal vector) and Y-axis (tHAT)
        Vector padLocalXAxis = pad.getNormalVector();
        Vector padLocalYAxis = pad.tHAT();

        // Calculate rotation matrix components (dot products give projection onto axes)
        double localX = relativeGlobalPos.dot(padLocalXAxis);
        double localY = relativeGlobalPos.dot(padLocalYAxis);
        double localVx = relativeGlobalVel.dot(padLocalXAxis);
        double localVy = relativeGlobalVel.dot(padLocalYAxis);

        // Orientation: Angle of lander's orientation vector relative to pad's local X-axis
        // Assuming lander's orientation is relative to global X-axis
        // We need to find the angle between lander's orientation vector and padLocalXAxis
        // This is complex and might need a dedicated rotation matrix or quaternion approach for 3D.
        // For 2D, if lander's orientation is just 'theta' in the global XY plane,
        // and padLocalXAxis is also in XY plane, then:
        double landerGlobalOrientation = lander.getOrientation(); // Assuming this is relative to global X
        double padLocalXAxisAngle = Math.atan2(padLocalXAxis.getY(), padLocalXAxis.getX());
        double relativeTheta = normalizeAngle(landerGlobalOrientation - padLocalXAxisAngle);

        // Angular velocity remains the same (scalar)
        double relativeOmega = lander.getAngularVelocity();

        return new LanderState(localX, localY, localVx, localVy, relativeTheta, relativeOmega);
    }

    private static void transformLocalToGlobal(LanderState localState, LanderObject lander, LandingPad pad) {
        // 1. Rotate back: Convert local 2D position/velocity to global 3D relative vectors
        Vector padLocalXAxis = pad.getNormalVector();
        Vector padLocalYAxis = pad.tHAT();

        // Reconstruct relative global position and velocity from local components
        // This assumes padLocalXAxis and padLocalYAxis are orthogonal unit vectors
        Vector relativeGlobalPos = padLocalXAxis.multiply(localState.x).add(padLocalYAxis.multiply(localState.y));
        Vector relativeGlobalVel = padLocalXAxis.multiply(localState.vx).add(padLocalYAxis.multiply(localState.vy));

        // 2. Translate back: Add pad's global position to get absolute global position
        Vector globalPos = relativeGlobalPos.add((Vector) pad.getPositionVector());
        Vector globalVel = relativeGlobalVel.add((Vector) pad.getVelocityVector());

        // Update LanderObject's position and velocity
        lander.setPosition((vectorInterface) globalPos);
        lander.setVelocity((vectorInterface) globalVel);

        // Update LanderObject's orientation and angular velocity
        // Reconstruct global orientation from relativeTheta and padLocalXAxisAngle
        double padLocalXAxisAngle = Math.atan2(padLocalXAxis.getY(), padLocalXAxis.getX());
        double globalOrientation = normalizeAngle(localState.theta + padLocalXAxisAngle);
        lander.setOrientation(globalOrientation);
        lander.setAngularVelocity(localState.omega);

        // Ensure Z-coordinate matches pad's Z
        lander.getPositionVector().setZ(pad.getPositionVector().getZ());
        lander.getVelocityVector().setZ(0);
    }
}
