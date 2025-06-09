package Physics_Engine.LandingController;

public class ControlInputs {
    public double thrust;           // Main thrust (0 to 10*g)
    public double torque;           // Torque (-1 to +1 rad/s^2)

    public ControlInputs(double thrust, double torque) {
        this.thrust = thrust;
        this.torque = torque;
    }
}