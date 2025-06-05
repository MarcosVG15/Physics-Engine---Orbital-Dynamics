package src.Physics_Engine.LandingController; // Corrected package declaration

import src.Physics_Engine.GeneralComponents.Interfaces.SpaceObject; // Import the SpaceObject interface from GeneralComponents
import src.Physics_Engine.GeneralComponents.Interfaces.vectorInterface; // Import the vectorInterface for position and velocity from GeneralComponents

/**
 * Represents the lander object with its physical properties and state.
 * This class implements the SpaceObject interface, allowing it to be part of a larger solar system simulation.
 */
public class LanderObject implements SpaceObject {
    private vectorInterface position; // The current position vector of the lander (e.g., x, y, z coordinates).
    private vectorInterface velocity; // The current velocity vector of the lander (e.g., vx, vy, vz components).
    private double mass; // The mass of the lander in kilograms.
    private String name; // The name of the lander (e.g., "Lunar Lander").
    private double orientation; // The current orientation (angle) of the lander in radians, typically around the Z-axis for 2D.
    private double angularVelocity; // The current angular velocity (rate of rotation) of the lander in radians per second.

    /**
     * Constructor for the LanderObject.
     *
     * @param position The initial position vector of the lander.
     * @param velocity The initial velocity vector of the lander.
     * @param mass The mass of the lander.
     * @param name The name of the lander.
     * @param orientation The initial orientation (angle) of the lander in radians.
     * @param angularVelocity The initial angular velocity of the lander in radians per second.
     */
    public LanderObject(vectorInterface position, vectorInterface velocity, double mass, String name, double orientation, double angularVelocity) {
        this.position = position;
        this.velocity = velocity;
        this.mass = mass;
        this.name = name;
        this.orientation = orientation;
        this.angularVelocity = angularVelocity;
    }

    @Override
    public vectorInterface getVelocityVector() {
        return velocity;
    }

    @Override
    public vectorInterface getPositionVector() {
        return position;
    }

    @Override
    public void setVelocity(vectorInterface v) {
        this.velocity = v;
    }

    @Override
    public void setPosition(vectorInterface v) {
        this.position = v;
    }

    @Override
    public void setName(String name) {
        this.name = name;
    }

    @Override
    public void print() {
        System.out.println("Lander: " + name);
        System.out.println("  Position: " + position.getX() + ", " + position.getY() + ", " + position.getZ());
        System.out.println("  Velocity: " + velocity.getX() + ", " + velocity.getY() + ", " + velocity.getZ());
        System.out.println("  Mass: " + mass);
        System.out.println("  Orientation (radians): " + orientation);
        System.out.println("  Angular Velocity (radians/s): " + angularVelocity);
    }

    @Override
    public double getMass() {
        return mass;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public boolean hasHitPlanet(SpaceObject astralObject, double Radius) {
        double distance = this.position.distance(astralObject.getPositionVector());
        return distance <= Radius;
    }

    /**
     * Gets the current orientation (angle) of the lander.
     * @return The orientation angle in radians.
     */
    public double getOrientation() {
        return orientation;
    }

    /**
     * Sets the orientation (angle) of the lander.
     * @param orientation The new orientation angle in radians.
     */
    public void setOrientation(double orientation) {
        this.orientation = orientation;
    }

    /**
     * Gets the current angular velocity of the lander.
     * @return The angular velocity in radians per second.
     */
    public double getAngularVelocity() {
        return angularVelocity;
    }

    /**
     * Sets the angular velocity of the lander.
     * @param angularVelocity The new angular velocity in radians per second.
     */
    public void setAngularVelocity(double angularVelocity) {
        this.angularVelocity = angularVelocity;
    }
}