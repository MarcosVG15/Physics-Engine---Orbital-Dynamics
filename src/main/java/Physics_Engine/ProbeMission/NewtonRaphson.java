package Physics_Engine.ProbeMission;

import Physics_Engine.GeneralComponents.AstralObject;
import Physics_Engine.GeneralComponents.ProbeObject;
import Physics_Engine.GeneralComponents.SolarSystem;
import Physics_Engine.GeneralComponents.Vector;
import Physics_Engine.ODESolverRK4.*;
import Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import Physics_Engine.GeneralComponents.Interfaces.vectorInterface;

import static Physics_Engine.RocketMissson.VARIABLES.STEPSIZE;

import java.util.ArrayList;
import java.util.Arrays;

public class NewtonRaphson {

    private final double TITAN_RADIUS = 2575 ;
    private final double ALPHA = 0.5;


    private ArrayList<double[]> PositionLog;
    private  ArrayList<vectorInterface> VelocityLog ;
    private ProbeObject probe ;


    public NewtonRaphson(){
        PositionLog = new ArrayList<>();
        VelocityLog = new ArrayList<>() ;

        SolarSystem solarSystem = new SolarSystem();
        this.probe = (ProbeObject) solarSystem.getSolarSystem().get(11);
        VelocityLog.add(probe.getVelocityVector());
         getDistanceEstimate();

        probe.setVelocity(new Vector(55.147313,-33.207901,-17.928841));
        VelocityLog.add(probe.getVelocityVector());
        getDistanceEstimate();

        System.out.println("Velocities ");
        for(vectorInterface object: VelocityLog){
            object.print(" ");
        }

        // System.out.println();
        // System.out.println("Position ");
        // for (double[] values : PositionLog){
        //     System.out.println(Arrays.toString(values));
        // }

    }


    public boolean ComputeNewtonRaphson(){

        boolean[] VelocityEquals = new boolean[3];
        double[] updatedVelocity  = new double[3] ;

        double[] currentVelocity = VelocityLog.get(VelocityLog.size()-1).getVector();
        double[] currentPosition = PositionLog.get(PositionLog.size()-1) ;
        double[] derivativeArray = getDerivative();

        double norm = Math.sqrt(Math.pow(derivativeArray[0],2)+Math.pow(derivativeArray[1],2)+Math.pow(derivativeArray[2],2));
        for(int i = 0 ; i<3 ; i++){
         updatedVelocity[i] = currentVelocity[i] - ALPHA*(currentPosition[i]/derivativeArray[i]);

            if (Math.abs(currentPosition[i]) < 10){
                VelocityEquals[i] = true;
            }
        }

        vectorInterface VelocityVector = new Vector(updatedVelocity[0] , updatedVelocity[1] , updatedVelocity[2]);
        VelocityLog.add(VelocityVector);
        this.probe.setVelocity(VelocityVector);

        getDistanceEstimate();




        //System.out.println("Velocities ");

        double[] DistanceArray = PositionLog.get(PositionLog.size()-1);
        System.out.print("Distance , " + DistanceArray[0] + " , "+ DistanceArray[1] +" , "+ DistanceArray[2]);

        // for(int  i = VelocityLog.size()-2 ; i<VelocityLog.size() ; i++){
            VelocityLog.get(VelocityLog.size()-1).print(", Velocity , ");
        //}

        // System.out.println();
        // System.out.println("Position ");

        // for(int  i = PositionLog.size()-2 ; i<PositionLog.size() ; i++){
        
      //  }


        return VelocityEquals[0]& VelocityEquals[1] & VelocityEquals[2] ;
    }



    public double[] getDerivative(){

        int sizeVelocityLog = VelocityLog.size();
        double[] currentVelocity = VelocityLog.get(sizeVelocityLog-1).getVector();
        double[] pastVelocity    = VelocityLog.get(sizeVelocityLog-2).getVector() ;

        int sizePosition = PositionLog.size();
        double[] currentPosition = PositionLog.get(sizePosition-1);
        double[] pastPosition    = PositionLog.get(sizePosition-2);

        double[] derivative = new double[3];

        for(int i = 0 ; i<3 ; i++){

            derivative[i] = ((currentPosition[i] - pastPosition[i])/(currentVelocity[i] - pastVelocity[i]));
        }


       // System.out.println("Derivative :  " + Arrays.toString(derivative));
        return derivative ;
    }



    /**
     * Method that performs one full rotation of the earth in order to compute the vector distance between Titan and the probe
     * If the probe has hit titan it terminates the process and return the modulus
     *
     * @return - returns the modulus of the distance between the probe and titan
     */
    public void getDistanceEstimate(){


        ProbeObject probeInstance = null ;
        AstralObject titan  = null ;

        SolarSystem S = new SolarSystem();
        ArrayList<SpaceObject> solarSystem = S.getSolarSystem();
        solarSystem.get(11).setVelocity(this.probe.getVelocityVector());

        for(int t = 0 ; t<31_536_000/STEPSIZE ; t++){
            probeInstance = (ProbeObject) solarSystem.get(11);
            titan = (AstralObject) solarSystem.get(8);
            //probe.print();

//            double[] DistanceArray = getDifferenceArray(titan.getPositionVector().getVector() ,probe.getPositionVector().getVector()) ;
//            System.out.println(DistanceArray[0] + " , "+ DistanceArray[1] +" , "+ DistanceArray[2]);

            if(probeInstance.hasHitPlanet(titan , TITAN_RADIUS)){ // if the probe is in the bounds of the moon

                //System.out.println("HAS HIT " + t);
                PositionLog.add(getDifferenceArray(titan.getPositionVector().getVector() ,probeInstance.getPositionVector().getVector()));
                return ;
            }

            AccelerationFunction acceleration = new AccelerationFunction();
            VelocityFunction velocity = new VelocityFunction();
            RK4_ODESolver odeSolver = new RK4_ODESolver();
            odeSolver.ComputeODE(0 , S, acceleration ,velocity);
        }

        PositionLog.add(getDifferenceArray(titan.getPositionVector().getVector() ,probeInstance.getPositionVector().getVector()));


//         System.out.println();
//        // System.out.println("HASN'T HIT :( ");
// //        System.out.println(Arrays.toString(getDifferenceArray(titan.getPositionVector().getVector() ,probe.getPositionVector().getVector())));
//         System.out.println();


    }


    public double[] getDifferenceArray(double[] Array1 , double[] Array2){
        double[] differenceArray = new double[3] ;

        for(int i = 0 ; i<3 ; i++){
            differenceArray[i] =  Array1[i] -Array2[i] ;
        }

        return differenceArray ;
    }

    public double getModulus ( vectorInterface vectorProbe , vectorInterface vectorTitan){

        double[] probeValues = vectorProbe.getVector();
        double[] titanValues = vectorTitan.getVector();

        double sum =0 ;

        for(int i = 0 ; i<3 ; i++){
            sum += Math.pow((probeValues[i]- titanValues[i]), 2);
        }

        return Math.sqrt(sum);
    }






}
