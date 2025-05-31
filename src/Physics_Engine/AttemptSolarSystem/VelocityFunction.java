package src.Physics_Engine.AttemptSolarSystem;


import src.Physics_Engine.Interfaces.SpaceObject;
import src.Physics_Engine.Interfaces.function;
import src.Physics_Engine.Interfaces.vectorInterface;

import java.util.ArrayList;

public class VelocityFunction implements function {
    @Override
    public vectorInterface computeDerivative(int planet, vectorInterface Vector, ArrayList<SpaceObject> solarSystem) {
        return Vector;
    }
}
