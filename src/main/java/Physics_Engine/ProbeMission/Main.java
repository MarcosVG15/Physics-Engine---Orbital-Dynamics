package src.main.java.Physics_Engine.ProbeMission;

public class Main {
    public static void main(String[] args){
        NewtonRaphson newtonRaphson = new NewtonRaphson();
        //newtonRaphson.getDistanceEstimate();
        while(!newtonRaphson.ComputeNewtonRaphson()){

        }
    }
}
