package src.Physics_Engine.SpectralDefferedInProgress;


import src.Physics_Engine.AttemptSolarSystem.AstralObjectRK4;

import java.util.ArrayList;

public class Main {
    public static void  main(String[] args){

        HeunTest();

    }

    public static void HeunTest(){
        for( int i = 0 ; i<10 ; i++){
            SolarSystem s = SolarSystem.getInstance();
            ArrayList<AstralObject> solarSystem = s.getSolarSystem();

            for (AstralObject a: solarSystem){
                a.print();
            }

            HEUN_2ORDER solver = new HEUN_2ORDER();
            AccelerationFunction acceleration = new AccelerationFunction();
            VelocityFunction velocity = new VelocityFunction();

            solver.ComputeODE(0,s,acceleration,velocity);
        }
    }
}
