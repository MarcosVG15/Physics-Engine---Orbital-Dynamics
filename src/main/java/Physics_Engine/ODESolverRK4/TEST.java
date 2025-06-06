package src.main.java.Physics_Engine.ODESolverRK4;

import src.main.java.Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import src.main.java.Physics_Engine.GeneralComponents.SolarSystem;

import java.util.ArrayList;

public class TEST {
    public static void main(String[] args){

        SolarSystem S = new SolarSystem();
        ArrayList<SpaceObject> solarSystem = S.getSolarSystem();


        long startTime = System.nanoTime();

       for(int t = 0 ; t<525600 ; t++){


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
