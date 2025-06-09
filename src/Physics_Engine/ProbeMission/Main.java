package src.Physics_Engine.ProbeMission;

import static src.Physics_Engine.RocketMissson.VARIABLES.STEPSIZE;

import java.util.Arrays;

public class Main {
    public static void main(String[] args){

        double[] stepSizes = new double[]{ 80 , 100 , 120 , 150 , 300, 600 } ; 
        for(int i = 0 ; i<stepSizes.length ; i++){
            double startTime = System.currentTimeMillis() ; 
            
            STEPSIZE = stepSizes[i] ; 

             NewtonRaphson newtonRaphson = new NewtonRaphson();
            //newtonRaphson.getDistanceEstimate();
                int step = 0 ;
                do{
                    System.out.print("Step : , "+ step + " , " );
                                step++;

                }
                while(!newtonRaphson.ComputeNewtonRaphson());

            double endTime = System.currentTimeMillis() ; 
            double duration = (endTime-startTime)/1000 ; 
            System.out.println("Duration : "+ duration + "For Step Size : "+ stepSizes[i]);
        }
        
        
    }
}
