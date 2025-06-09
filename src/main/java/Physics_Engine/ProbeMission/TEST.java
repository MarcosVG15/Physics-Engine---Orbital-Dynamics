package Physics_Engine.ProbeMission;

import Physics_Engine.GeneralComponents.Interfaces.vectorInterface;
import Physics_Engine.GeneralComponents.SolarSystem;
import Physics_Engine.ODESolverRK4.AccelerationFunction;
import Physics_Engine.ODESolverRK4.RK4_ODESolver;
import Physics_Engine.ODESolverRK4.VelocityFunction;
import Physics_Engine.GeneralComponents.Interfaces.SpaceObject;

import java.util.ArrayList;

import static Physics_Engine.RocketMissson.VARIABLES.STEPSIZE;

public class TEST {
    public static void main(String[] agrs){
        SolarSystem S = new SolarSystem() ;

        ArrayList<SpaceObject> solarSystem = S.getSolarSystem();


        for(int t = 0 ; t<31_536_000/STEPSIZE ; t++){


            AccelerationFunction acceleration = new AccelerationFunction();
            VelocityFunction velocity = new VelocityFunction();
            RK4_ODESolver odeSolver = new RK4_ODESolver();

            odeSolver.ComputeODE(8 , S, acceleration ,velocity);

        }

        double[] positionProbe = solarSystem.get(11).getPositionVector().getVector() ; // gets the probe
        double[] positionTitan = solarSystem.get(8).getPositionVector().getVector() ; // gets the titan

        double xDifference = positionProbe[0] - positionTitan[0] ;
        double yDifference = positionProbe[1] - positionTitan[1] ;
        double zDifference = positionProbe[2] - positionTitan[2] ;

        System.out.println("For Step size "+ STEPSIZE+ "   Accuracy :  "+ xDifference + " , "+ yDifference +" , "+ zDifference);
    }
}
