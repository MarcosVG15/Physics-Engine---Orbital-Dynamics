package src.Physics_Engine.RocketMissson;

import src.Physics_Engine.GeneralComponents.Interfaces.vectorInterface;
import src.Physics_Engine.GeneralComponents.Vector;

import java.util.Random;


public class genomeVector extends Vector {
    private double Fitness ;


    public genomeVector(double x , double  y , double z){
        super(x , y , z) ;
    }

    public double getFitness(){
        return Fitness ;
    }
    public void setFitness(double fitness){
        this.Fitness =fitness ;
    }

    public void mutate(){

        Random random = new Random();
        double[] currentVelocityVectorArray =super.getVector();

        for(int i = 0 ; i<3 ; i++){
            if(random.nextDouble()*100 <2){
                currentVelocityVectorArray[i] += random.nextDouble()*1 - 0.5 ;
            }
        }
        super.setVector(new Vector(currentVelocityVectorArray[0] , currentVelocityVectorArray[1] , currentVelocityVectorArray[2]));

    }



}
