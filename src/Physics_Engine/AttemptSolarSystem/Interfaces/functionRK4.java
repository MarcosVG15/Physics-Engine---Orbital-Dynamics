package src.Physics_Engine.AttemptSolarSystem.Interfaces;


import src.Physics_Engine.AttemptSolarSystem.AstralObjectRK4;

import java.util.ArrayList;

public interface functionRK4 {
    public vectorInterfaceRK4 computeDerivative(int planet, vectorInterfaceRK4 Vector, ArrayList<AstralObjectRK4> solarSystem) ;

}
