package src.main.java.Physics_Engine.LandingController;

import static src.main.java.Physics_Engine.LandingController.Constants.GRAVITY_TITAN;
import java.util.ArrayList;
import src.main.java.Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import src.main.java.Physics_Engine.GeneralComponents.Interfaces.function;
import src.main.java.Physics_Engine.GeneralComponents.Interfaces.vectorInterface;
import src.main.java.Physics_Engine.GeneralComponents.Vector;

public class LanderPhysicsFunction implements function {
    private LanderController controller;
    private WindModel windModel;

    public LanderPhysicsFunction(LanderController controller, WindModel windModel) {
        this.controller = controller;
        this.windModel = windModel;
    }

    @Override
    public vectorInterface computeDerivative(int i, vectorInterface position, ArrayList<SpaceObject> solarSystem) {
        // Assuming the lander is the object at index 'i'
        if (!(solarSystem.get(i) instanceof LanderObject)) {
            throw new IllegalArgumentException("Expected LanderObject at index " + i);
        }

        LanderObject lander = (LanderObject) solarSystem.get(i);

        // Create a LanderState from the LanderObject for the controller
        LanderState landerState = new LanderState(
            lander.getPositionVector().getX(),
            lander.getPositionVector().getY(),
            lander.getVelocityVector().getX(),
            lander.getVelocityVector().getY(),
            lander.getOrientation(),
            lander.getAngularVelocity()
        );
        // Get control inputs from the controller
        ControlInputs inputs = controller.calculateControlInputs(landerState);

        // Wind model commented out as per user request
        // double time = 0; // Placeholder, will need to be passed from RK4_ODESolver
        // double wind_x = windModel.getWindX(time);
        // double wind_y = windModel.getWindY(time);
        double wind_x = 0; // Set to 0 for now
        double wind_y = 0; // Set to 0 for now

        // Calculate derivatives for position, velocity, orientation, and angular velocity
        // These are the "rates of change" for each state variable
        double dx_dt = lander.getVelocityVector().getX();
        double dy_dt = lander.getVelocityVector().getY();
        double dvx_dt = inputs.thrust * Math.sin(lander.getOrientation()) + wind_x;
        double dvy_dt = inputs.thrust * Math.cos(lander.getOrientation()) - GRAVITY_TITAN + wind_y;
        // The 'function' interface expects a single vectorInterface return, representing acceleration.
        // We will handle theta and omega integration separately in the simulation loop.
        return new Vector(dvx_dt, dvy_dt, 0); // Z-component is unused for 2D acceleration
    }
}