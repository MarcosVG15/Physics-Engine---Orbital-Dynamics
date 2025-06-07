package src.Physics_Engine.ProbeMission;

import src.Physics_Engine.GeneralComponents.SolarSystem;
import src.Physics_Engine.ODESolverRK4.AccelerationFunction;
import src.Physics_Engine.ODESolverRK4.RK4_ODESolver;
import src.Physics_Engine.ODESolverRK4.VelocityFunction;
import src.Physics_Engine.GeneralComponents.Interfaces.SpaceObject;

import java.util.ArrayList;

import static src.Physics_Engine.RocketMissson.VARIABLES.STEPSIZE;

public class TEST {
    public static void main(String[] agrs){
        SolarSystem S = new SolarSystem() ;

        ArrayList<SpaceObject> solarSystem = S.getSolarSystem();


        for(int t = 0 ; t<31_536_000/STEPSIZE ; t++){


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
