package src.Physics_Engine.ODESolverRK4;




import src.Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import src.Physics_Engine.GeneralComponents.Interfaces.function;
import src.Physics_Engine.GeneralComponents.Interfaces.vectorInterface;
import src.Physics_Engine.GeneralComponents.Vector;

import java.util.ArrayList;

public class AccelerationFunction implements function {
    public static final double G = 6.67430e-20 ; //we are working with km ;


    @Override
    public vectorInterface computeDerivative(int planet, vectorInterface VectorPosition, ArrayList<SpaceObject> solarSystem){
        double [] accelerationValues = new double[3];


        for(int i = 0 ; i<accelerationValues.length ;i++){

            double summation = 0;

            for(int j = 0 ; j<solarSystem.size();j++){
                if(j==planet){
                    continue;
                }

                SpaceObject current = solarSystem.get(j);
                double[] planetAPosition = VectorPosition.getVector();
                double[] currentPosition = current.getPositionVector().getVector();

                double modulus = getModulus(VectorPosition,current.getPositionVector());
                double MassDividedModulus = (planetAPosition[i] - currentPosition[i])/Math.pow((modulus),3);

                summation+= G*MassDividedModulus*(current.getMass());

            }
            accelerationValues[i] = -summation;

        }

        Vector acceleration = new Vector(accelerationValues[0],accelerationValues[1],accelerationValues[2]);

        return acceleration;
    }

    private double getModulus(vectorInterface v1 , vectorInterface v2){
        double modulus ;
        double[] valueV1 = v1.getVector();
        double[] valueV2 = v2.getVector();

        double differenceSum = 0 ;

        for(int i = 0 ; i<valueV2.length ; i++){

            differenceSum+=Math.pow(valueV1[i]-valueV2[i],2);

        }

        modulus = Math.sqrt(differenceSum);

        return modulus;
    }


}
