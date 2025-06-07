package Physics_Engine.ProbeMission;

import Physics_Engine.GeneralComponents.SolarSystem;
import Physics_Engine.ODESolverRK4.AccelerationFunction;
import Physics_Engine.ODESolverRK4.RK4_ODESolver;
import Physics_Engine.ODESolverRK4.VelocityFunction;
import Physics_Engine.GeneralComponents.Interfaces.SpaceObject;

import java.util.ArrayList;

public class TEST {
    public static void main(String[] agrs){
        SolarSystem S = new SolarSystem() ;

        ArrayList<SpaceObject> solarSystem = S.getSolarSystem();


        for(int t = 0 ; t<525600 ; t++){


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
