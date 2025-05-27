package src.Physics_Engine.AttemptSolarSystem;

import java.util.ArrayList;

public class TEST {
    public static void main(String[] args){

        SolarSystemRK4 S = src.Physics_Engine.AttemptSolarSystem.SolarSystemRK4.getInstance();

        ArrayList<AstralObjectRK4> solarSystem = S.getSolarSystem();


       for(int t = 0 ; t<10000 ; t++){

           solarSystem.get(1).print();

           AccelerationFunctionRK4 acceleration = new AccelerationFunctionRK4();
           VelocityFunctionRK4 velocity = new VelocityFunctionRK4();
           RK4_ODESolver odeSolver = new RK4_ODESolver();

           odeSolver.ComputeODE(t , S, acceleration ,velocity);




       }



    }
}
