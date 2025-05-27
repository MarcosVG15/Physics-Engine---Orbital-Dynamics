package src.Physics_Engine.AttemptSolarSystem;




import src.Physics_Engine.AttemptSolarSystem.Interfaces.functionRK4;
import src.Physics_Engine.AttemptSolarSystem.Interfaces.vectorInterfaceRK4;

import java.util.ArrayList;

public class AccelerationFunctionRK4 implements functionRK4 {
    public static final double G = 6.67430e-11;


    @Override
    public vectorInterfaceRK4 computeDerivative(int planet, vectorInterfaceRK4 VectorPosition, ArrayList<AstralObjectRK4> solarSystem){
        double [] accelerationValues = new double[3];


        for(int i = 0 ; i<accelerationValues.length ;i++){

            double summation = 0;

            for(int j = 0 ; j<solarSystem.size();j++){
                if(j==planet){
                    continue;
                }
                AstralObjectRK4 current = solarSystem.get(j);

                double modulus = getModulus(VectorPosition,current.getPositionVector());
                double MassDividedModulus = current.getMass()/Math.pow(Math.abs(modulus),3);

                double[] planetAPosition = VectorPosition.getVector();
                double[] currentPosition = current.getPositionVector().getVector();

                summation+= MassDividedModulus*(planetAPosition[i] - currentPosition[i]);

            }
            accelerationValues[i] = -G*summation;

        }

        VectorRK4 acceleration = new VectorRK4(accelerationValues[0],accelerationValues[1],accelerationValues[2]);

        return acceleration;
    }

    private double getModulus(vectorInterfaceRK4 v1 , vectorInterfaceRK4 v2){
        double modulus = 0;
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
