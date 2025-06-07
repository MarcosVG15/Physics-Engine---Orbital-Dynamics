package Physics_Engine.LandingController;

public final class Constants {
    // Physical Constants
    public static final double GRAVITY_TITAN = 1.352; // m/s^2
    public static final double THRUST_MAX = 10.0 * GRAVITY_TITAN; // umax = 10 * g
    public static final double TORQUE_MAX = 1.0; // rad/s^2
    public static final double SPACESHIP_MASS = 50000; 

    // Success Criteria
    public static final double DELTA_X = 0.1; // m (max horizontal position at landing)
    public static final double DELTA_THETA = 0.02; // rad (max orientation deviation at landing)
    public static final double EPSILON_X = 0.1; // m/s (max horizontal velocity at landing)
    public static final double EPSILON_Y = 0.1; // m/s (max vertical velocity at landing)
    public static final double EPSILON_THETA = 0.01; // rad/s (max angular velocity at landing)
    public static final double KM_TO_M = 1000.0; // Kilometers to Meters conversion factor

    // Private constructor to prevent instantiation
    private Constants() {
        // Utility class
    }
}