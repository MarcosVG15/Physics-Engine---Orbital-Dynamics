package Physics_Engine.LandingController;

import Physics_Engine.GeneralComponents.Interfaces.vectorInterface;
import Physics_Engine.GeneralComponents.Vector;

public class LanderState {
    public double x, y;        // Position (m)
    public double vx, vy;      // Velocity (m/s)
    public double theta;       // Orientation (rad)
    public double omega;       // Angular velocity (rad/s)

    public LanderState(double x, double y, double vx, double vy, double theta, double omega) {
        this.x = x;
        this.y = y;
        this.vx = vx;
        this.vy = vy;
        this.theta = theta;
        this.omega = omega;
    }

    public vectorInterface getPositionVector() {
        return new Vector(x, y, 0); // Assuming 2D, Z is 0
    }

    public vectorInterface getVelocityVector() {
        return new Vector(vx, vy, 0); // Assuming 2D, Z is 0
    }

    public double getOrientation() {
        return theta;
    }

    public double getAngularVelocity() {
        return omega;
    }


    // Optional: Add a copy constructor or clone method if your solver needs immutable states
    public LanderState clone() {
        return new LanderState(this.x, this.y, this.vx, this.vy, this.theta, this.omega);
    }
}