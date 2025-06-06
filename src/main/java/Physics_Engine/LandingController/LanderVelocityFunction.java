package src.main.java.Physics_Engine.LandingController;

import java.util.ArrayList;
import src.main.java.Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import src.main.java.Physics_Engine.GeneralComponents.Interfaces.function;
import src.main.java.Physics_Engine.GeneralComponents.Interfaces.vectorInterface;

public class LanderVelocityFunction implements function {

    @Override
    public vectorInterface computeDerivative(int i, vectorInterface position, ArrayList<SpaceObject> solarSystem) {
        // Assuming the lander is the object at index 'i'
        if (!(solarSystem.get(i) instanceof LanderObject)) {
            throw new IllegalArgumentException("Expected LanderObject at index " + i);
        }

        LanderObject lander = (LanderObject) solarSystem.get(i);

        // The derivative of position is velocity
        return lander.getVelocityVector();
    }
}