package src.Physics_Engine.RocketMissson;

import src.Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import src.Physics_Engine.GeneralComponents.SolarSystem;

import java.util.ArrayList;

public class TEST {
    public static void main(String[] args){
        SolarSystem solarSystem = new SolarSystem();
        ArrayList<SpaceObject> solarSystemArrayList = solarSystem.getSolarSystem() ;

        OptimalStepComputation stepComputation = new OptimalStepComputation() ;
        ArrayList<SpaceObject> snapshotSolarSystem = stepComputation.getSnapshot(solarSystemArrayList);

        for(SpaceObject object : snapshotSolarSystem){
            object.print();
        }

    }


}
