package src.Physics_Engine.ProbeMission;

import java.util.Arrays;

public class Main {
    public static void main(String[] args){
        NewtonRaphson newtonRaphson = new NewtonRaphson();
        //newtonRaphson.getDistanceEstimate();
        while(!newtonRaphson.ComputeNewtonRaphson()){

        }
    }
}
