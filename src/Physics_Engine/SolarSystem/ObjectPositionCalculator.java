package src.Physics_Engine.SolarSystem;

import java.util.ArrayList;
import java.util.Arrays;


public class ObjectPositionCalculator {

    private double stepSize;
    private static int currentCoordinateIndex ;
    private ArrayList<AstralObject> solarSystem ;

    public ObjectPositionCalculator(ArrayList<AstralObject> solarSystem, double stepSize) {
        this.stepSize = stepSize ;
        this.solarSystem = solarSystem ;
        this.currentCoordinateIndex= 0;

    }

    public ArrayList<AstralObject> getNextStep(double nextStep) {

        for (int i = 1; i < solarSystem.size(); i++) {

            AstralObject currentAstralObject  = solarSystem.get(i) ;

            SpeedFunction speedFunction = new SpeedFunction(solarSystem , i);
            CoordinateFunction coordinateFunction = new CoordinateFunction() ;

            double[][] speeds  = currentAstralObject.getSpecificVelocities(currentCoordinateIndex);
            double[][] coordinates = currentAstralObject.getSpecificCoordinates(currentCoordinateIndex);
            System.out.println(Arrays.toString(coordinates[0]));

            Adams_Bashforth_Solver adamsBashforthSolver = new Adams_Bashforth_Solver() ;

            double[] updatedSpeed = adamsBashforthSolver.AB4(stepSize , nextStep , coordinates[0]
                                    ,currentAstralObject.getSpecificCoordinates(3) ,speedFunction ) ;

            double[] updatedCoordinates = adamsBashforthSolver.AB4(stepSize , nextStep , speeds[0]
                                        , currentAstralObject.getSpecificVelocities(3) , coordinateFunction) ;


            solarSystem.get(i).addVelocities(updatedSpeed);
            solarSystem.get(i).addCoordinate(updatedCoordinates);
        }
        currentCoordinateIndex++;
        return solarSystem ;
    }

}


