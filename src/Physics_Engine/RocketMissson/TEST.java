package src.Physics_Engine.RocketMissson;

import src.Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import src.Physics_Engine.GeneralComponents.SolarSystem;
import src.Physics_Engine.GeneralComponents.Vector;

import java.util.ArrayList;

public class TEST {
    public static void main(String[] args){
        SolarSystem solarSystem = new SolarSystem();

        OptimalStepComputation stepComputation = new OptimalStepComputation(solarSystem) ;
        stepComputation.getBestVectorForStep();
    }


}
