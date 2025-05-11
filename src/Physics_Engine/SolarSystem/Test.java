package src.Physics_Engine.SolarSystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Test {
    public static void main(String[] args){
        Adams_Bashforth_Solver solver = new Adams_Bashforth_Solver();
        SolarSystem solar = new SolarSystem();
        List<AstralObject> solarSystem = solar.getSolar();

        ODEFunction dv = new VelocityDerivative();
        PositionDerivative dx = new PositionDerivative();

        int h = 1;
        int t = 0;
        int steps = 0;
        while(steps < 100) {
            computeAcceleration(solarSystem);
            System.out.println("t = " + t + "-----------------------------------------------------------------------" );
            for (int i = 0; i<solarSystem.size(); i++){
                AstralObject object = solarSystem.get(i);
                System.out.println(object.getName()+ ": coordinates: " + Arrays.toString(object.getCoordinates())
                        + ", velocities: "+ Arrays.toString(object.getVelocities()) +"Accelerations: "+ Arrays.toString(object.getAcceleration()));
            }
            for (AstralObject object : solarSystem) {
                object.setVelocities(solver.AB4(h, t, object.getVelocities(), object.getPastCoordinates(), object.getAcceleration(), dv));
                object.setCoordinates(solver.AB4(h, t, object.getCoordinates(), object.getPastCoordinates(), object.getVelocities(), dx));
            }





            t += h;
            steps+=1;

        }
    }

    public static void computeAcceleration(List<AstralObject> solarSystem){
        for (AstralObject object : solarSystem){
            object.setAcceleration(solarSystem);
        }
    }

}
