package src.Physics_Engine.AttemptSolarSystem;

import java.util.ArrayList;

public class TEST {
    public static void main(String[] args){

        SolarSystem S = SolarSystem.getInstance();

        ArrayList<AstralObject> solarSystem = S.getSolarSystem();

       for(int t = 0 ; t<10 ; t++){

           for (AstralObject a: solarSystem){
               a.print();
           }
           AccelerationFunction acceleration = new AccelerationFunction();
           VelocityFunction velocity = new VelocityFunction();
           RK4_ODESolver odeSolver = new RK4_ODESolver();

           odeSolver.ComputeODE(t , S, acceleration ,velocity);




       }



    }
}
