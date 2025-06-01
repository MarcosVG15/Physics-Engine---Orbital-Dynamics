package src.Physics_Engine.RocketMissson;

import src.Physics_Engine.GeneralComponents.Interfaces.vectorInterface;
import src.Physics_Engine.GeneralComponents.Vector;

import java.util.Random;

public class OptimalStepComputation {

    private vectorInterface spaceShipPosition ;
    private vectorInterface titanPosition ;

    public OptimalStepComputation(){

    }

    public void mutate(vectorInterface currentVelocityVector){

        Random random = new Random();
        double[] currentVelocityVectorArray = currentVelocityVector.getVector();

        for(int i = 0 ; i<3 ; i++){
            if(random.nextDouble()*100 <2){
                currentVelocityVectorArray[i] += random.nextDouble()*5 - 2.5 ;
            }
        }
        currentVelocityVector.setVector(new Vector(currentVelocityVectorArray[0] , currentVelocityVectorArray[1] , currentVelocityVectorArray[2]));

    }

}
