package src.Physics_Engine.ODESolverRK4;

import src.Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import src.Physics_Engine.GeneralComponents.Interfaces.vectorInterface;
import src.Physics_Engine.GeneralComponents.SolarSystem;
import src.Physics_Engine.GeneralComponents.SpaceShip;
import src.Physics_Engine.GeneralComponents.Thrust;
import src.Physics_Engine.GeneralComponents.Vector;

import java.util.ArrayList;

import static src.Physics_Engine.RocketMissson.VARIABLES.STEPSIZE;

public class TEST {
    public static void main(String[] args){

        SolarSystem S = new SolarSystem();
        ArrayList<SpaceObject> solarSystem = S.getSolarSystem();


        long startTime = System.nanoTime();

       for(int t = 0 ; t<31_536_000/STEPSIZE ; t++){


           for(SpaceObject object : solarSystem){
               object.print();
           }

           AccelerationFunction acceleration = new AccelerationFunction();
           VelocityFunction velocity = new VelocityFunction();
           RK4_ODESolver odeSolver = new RK4_ODESolver();




           odeSolver.ComputeODE(8 , S, acceleration ,velocity);

       }
       long endTime = System.nanoTime();
        long durationInNs = endTime - startTime;
        System.out.println("Execution time: " + (durationInNs / 1_000_000_000.0) + " s");




    }
}
