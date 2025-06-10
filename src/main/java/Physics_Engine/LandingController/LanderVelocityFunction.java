package Physics_Engine.LandingController;

import Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import Physics_Engine.GeneralComponents.Interfaces.function;
import Physics_Engine.GeneralComponents.Interfaces.vectorInterface;
import java.util.ArrayList;

public class LanderVelocityFunction implements function {

    @Override
    public vectorInterface computeDerivative(int i, vectorInterface position, ArrayList<SpaceObject> solarSystem) {
        // Assuming the lander is the object at index 'i'
        if (!(solarSystem.get(i) instanceof LanderObject)) {
            throw new IllegalArgumentException("Expected LanderObject at index " + i);
        }

        LanderObject lander = (LanderObject) solarSystem.get(i);


        return lander.getVelocityVector();
    }
}