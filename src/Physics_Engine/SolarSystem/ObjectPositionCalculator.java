package src.Physics_Engine.SolarSystem;

import java.util.ArrayList;
import java.util.Arrays;


public class ObjectPositionCalculator {

    private double stepSize;
    private static int currentCoordinateIndex ;
    private ArrayList<AstralObject> tempSolarSystem ;

    public ObjectPositionCalculator(ArrayList<AstralObject> solarSystem, double stepSize) {
        this.stepSize = stepSize ;
        this.tempSolarSystem = solarSystem ;
        this.currentCoordinateIndex= 1;

    }

    public ArrayList<AstralObject> getNextStep(double nextStep) {

        for (int i = 1; i < tempSolarSystem.size(); i++) {

            AstralObject currentAstralObject  = tempSolarSystem.get(i) ;

            SpeedFunction speedFunction = new SpeedFunction(tempSolarSystem , i);
            CoordinateFunction coordinateFunction = new CoordinateFunction() ;

            double[][] speeds  = currentAstralObject.getSpecificVelocities(currentCoordinateIndex);
            double[][] coordinates = currentAstralObject.getSpecificCoordinates(currentCoordinateIndex);
            System.out.println(Arrays.toString(coordinates[0]));

            Adams_Bashforth_Solver adamsBashforthSolver = new Adams_Bashforth_Solver() ;

            double[] updatedSpeed = adamsBashforthSolver.AB4(stepSize , nextStep , coordinates[0]
                                    ,currentAstralObject.getSpecificCoordinates(3) ,speedFunction ) ;

            double[] updatedCoordinates = adamsBashforthSolver.AB4(stepSize , nextStep , speeds[0]
                                        , currentAstralObject.getSpecificVelocities(3) , coordinateFunction) ;


            tempSolarSystem.get(i).addVelocities(updatedSpeed);
            tempSolarSystem.get(i).addCoordinate(updatedCoordinates);
        }
        currentCoordinateIndex++;
        return tempSolarSystem ;
    }

}


