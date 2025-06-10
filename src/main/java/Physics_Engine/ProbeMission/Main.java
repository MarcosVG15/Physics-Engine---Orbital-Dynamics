package Physics_Engine.ProbeMission;

import static Physics_Engine.RocketMissson.VARIABLES.STEPSIZE;
import java.util.Arrays;

public class Main {
    public static void main(String[] args){

        double[] stepSizes = new double[]{ 30, 60 , 80, 100, 120 , 150 , 300, 600 } ;
        //double[] learningRates = new double[]{ 0.15 ,0.2 , 0.25 ,0.3 ,0.35, 0.4 ,0.45, 0.5 ,0.55, 0.6} ; 
        for(int i = 0 ; i<stepSizes.length ; i++){
            double startTime = System.currentTimeMillis() ; 
            
            STEPSIZE = stepSizes[i] ; 

             NewtonRaphson newtonRaphson = new NewtonRaphson();
            //newtonRaphson.getDistanceEstimate();
                int step = 0 ;
                do{
                    step++;
                }
                while(!newtonRaphson.ComputeNewtonRaphson(step));


            double endTime = System.currentTimeMillis() ;
            double duration = (endTime-startTime)/1000 ; 
            System.out.println("Duration : "+ duration + "For Step Size : "+ stepSizes[i]);
        }
        
        
    }
}
