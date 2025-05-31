package src.Physics_Engine.ProbeMission;

import src.Physics_Engine.AttemptSolarSystem.AccelerationFunction;
import src.Physics_Engine.AttemptSolarSystem.RK4_ODESolver;
import src.Physics_Engine.AttemptSolarSystem.SolarSystemRK4;
import src.Physics_Engine.AttemptSolarSystem.VelocityFunction;
import src.Physics_Engine.Interfaces.SpaceObject;

import java.util.ArrayList;

public class TEST {
    public static void main(String[] agrs){
        SolarSystem S = new SolarSystem() ;

        ArrayList<SpaceObject> solarSystem = S.getSolarSystem();


        for(int t = 0 ; t<52560 ; t++){


            for(SpaceObject object : solarSystem){
                object.print();
            }

            AccelerationFunction acceleration = new AccelerationFunction();
            VelocityFunction velocity = new VelocityFunction();
            RK4_ODESolver odeSolver = new RK4_ODESolver();

            odeSolver.ComputeODE(8 , S, acceleration ,velocity);

        }
    }
}
