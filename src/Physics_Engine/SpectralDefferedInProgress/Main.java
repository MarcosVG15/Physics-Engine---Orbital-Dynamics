package src.Physics_Engine.SpectralDefferedInProgress;


import src.Physics_Engine.Interfaces.SpaceObject;

import java.util.ArrayList;
import java.util.Arrays;

public class Main {
    public static void  main(String[] args){

        QIJMatrix matrix = new QIJMatrix(2 , 4);
        System.out.println(Arrays.toString(matrix.getAllTs()));
        System.out.println(matrix.LagrangePolynomial(2.2 , 2.6 ,matrix.getAllTs()));

        double[][] matrixQij = matrix.QijMatrix(10);

        for(int i = 0 ; i<matrixQij.length ; i++){
            System.out.println(Arrays.toString(matrixQij[i]));
        }

    }

    public static void HeunTest(){
        for( int i = 0 ; i<10 ; i++){
            SolarSystem s = SolarSystem.getInstance();
            ArrayList<SpaceObject> solarSystem = s.getSolarSystem();

            for (SpaceObject a: solarSystem){
                a.print();
            }

            HEUN_2ORDER solver = new HEUN_2ORDER();
            AccelerationFunction acceleration = new AccelerationFunction();
            VelocityFunction velocity = new VelocityFunction();

            solver.ComputeODE(0,s,acceleration,velocity);
        }
    }
}
