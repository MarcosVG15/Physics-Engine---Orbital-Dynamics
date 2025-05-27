package src.Physics_Engine.AttemptSolarSystem;


import src.Physics_Engine.AttemptSolarSystem.Interfaces.functionRK4;
import src.Physics_Engine.AttemptSolarSystem.Interfaces.vectorInterfaceRK4;

import java.util.ArrayList;

public class VelocityFunctionRK4 implements functionRK4 {
    @Override
    public vectorInterfaceRK4 computeDerivative(int planet, vectorInterfaceRK4 Vector, ArrayList<AstralObjectRK4> solarSystem) {
        return Vector;
    }
}
