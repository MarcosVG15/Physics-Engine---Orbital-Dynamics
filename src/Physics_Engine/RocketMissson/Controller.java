package src.Physics_Engine.RocketMissson;

import src.Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import src.Physics_Engine.GeneralComponents.Interfaces.vectorInterface;
import src.Physics_Engine.GeneralComponents.SolarSystem;
import src.Physics_Engine.GeneralComponents.SpaceShip;
import src.Physics_Engine.GeneralComponents.Thrust;
import src.Physics_Engine.GeneralComponents.Vector;
import src.Physics_Engine.ODESolverRK4.AccelerationFunction;
import src.Physics_Engine.ODESolverRK4.RK4_ODESolver;
import src.Physics_Engine.ODESolverRK4.VelocityFunction;

import java.util.ArrayList;
import java.util.Arrays;

import static src.Physics_Engine.RocketMissson.VARIABLES.STEPSIZE;

public class Controller {

    public double radiusEarth  = 6370 ;

    public Controller(){
        setTargetsAndRunSimulation() ;

    }



    /**
     * Class that takes in the max fuel is can use and the position coordinate that is limited to the orbit of the earth
     * such that we can optimise this specific part based on the other parameters too .
     *
     * has to start at velocity 0 and uses the thrust based on fuel to move

     */

    public void setTargetsAndRunSimulation(){

        SolarSystem solarSystemObject = new SolarSystem() ;
        ArrayList<SpaceObject > solarSystem = solarSystemObject.getSolarSystem() ;

        vectorInterface earthStartPosition = solarSystem.get(3).getPositionVector() ;
        vectorInterface titanPosition = new Vector(-603483.9688470364, -2797969.1013597567, 1455235.6071535358) ;  //COMPUTE THE END POSITION OF TITAN IN A YEAR



        vectorInterface normalVector = getDistance(earthStartPosition , titanPosition) ;

        vectorInterface startPosition = getScale(normalVector , radiusEarth) ;
        solarSystem.get(3).setPosition(startPosition);

        vectorInterface endPosition   = getScale(normalVector , radiusEarth+35_786) ;

        GradientDescentStage1 gradientDescentStage1 = new GradientDescentStage1(solarSystem , endPosition , 700_000 ,1) ;
        Thrust[] arrayOfThrust = gradientDescentStage1.gradientDescent() ; // COMPUTE STAGE 1 of gradient Descent

        for( int i = 0 ; i<arrayOfThrust.length ; i++){
            arrayOfThrust[i].print();
        }


//        run(solarSystemObject , arrayOfThrust , titanPosition);
//        GradientDescentStage2 gradientDescentStage2 = new GradientDescentStage2(solarSystemObject.getSolarSystem() , titanPosition , new Vector(23 , 0 , 0 ) , 500_000 , 3);
//        Thrust[] arrayOfThrust2 = gradientDescentStage2.gradientDescent() ;
//
//
//
//
//        for( int i = 0 ; i<arrayOfThrust.length ; i++){
//            arrayOfThrust[i].print();
//        }
//
//        for( int i = 0 ; i<arrayOfThrust2.length ; i++){
//            arrayOfThrust2[i].print();
//        }


    }

    public void run( SolarSystem SolarSystemObj , Thrust[] arrayOfThrustUpdate , vectorInterface target){
        boolean shouldTerminate ;

        ArrayList<SpaceObject> solarSystem = SolarSystemObj.getSolarSystem() ;

        shouldTerminate = false;
        System.out.println(" ");

         double steps = 0 ;
        while(steps<31_536_000/STEPSIZE){

            AccelerationFunction acceleration = new AccelerationFunction();
            VelocityFunction velocity = new VelocityFunction();
            RK4_ODESolver odeSolver = new RK4_ODESolver();


            for(Thrust thrust : arrayOfThrustUpdate){

                double startTime  = thrust.getStartTime();
                double endTime = startTime+thrust.getDuration() ;

                if(startTime <= steps && endTime>= steps){

                    SpaceShip spaceShip = (SpaceShip) solarSystem.get(12);

                    vectorInterface VelocityVector = spaceShip.getVelocityVectorPasByValue();
                    VelocityVector.add(scale(thrust , STEPSIZE/spaceShip.getMass()));

                    solarSystem.get(12).setVelocity(new Vector(VelocityVector.getX(), VelocityVector.getY(), VelocityVector.getZ() ));
                }

            }
            odeSolver.ComputeODE(3 , SolarSystemObj, acceleration ,velocity);

            vectorInterface endPosition = solarSystem.get(12).getPositionVector();

            vectorInterface distance = new Vector(target.getX()-endPosition.getX() , target.getY() - endPosition.getY() , target.getZ() - endPosition.getZ() );
            shouldTerminate = checkIfDistanceIsZero(distance);

            steps++;

        }
    }

    /**
     * This method will determine the stopping condition for the gradient descent which will happen if the rocket is
     * within 10 km of the target , this can be tweaked later on for more accuracy
     * @param distance  - this is the vector that is given by the simulation
     * @return - will return true if the rocket is within a 10 km sphere around to the target
     */
    public boolean checkIfDistanceIsZero(vectorInterface distance){

        boolean[] checkDistance  = new boolean[3];
        double[] distanceArray = distance.getVector() ;

        for(int i = 0 ; i<3 ; i++){
            if(Math.abs(distanceArray[i]) <=10){ // ten kilometers from target , i can change that later
                checkDistance[i] = true ;
            }
        }
        return checkDistance[0]& checkDistance[1] & checkDistance[2] ;
    }

    private vectorInterface scale(vectorInterface vector , double scale){
        double[] vectorArray = vector.getVector() ;
        double[] scaledArray = new double[vectorArray.length];

        for(int i = 0 ; i<3 ; i++){
            scaledArray[i] = scale*vectorArray[i] ;
        }
        return new Vector(scaledArray[0] , scaledArray[1] , scaledArray[2]);
    }

    public vectorInterface getDistance(vectorInterface earthPosition , vectorInterface titanPosition){

        double[] distanceOfEarthToTitan  = new double[3] ;

        double[] earthCoordinate = earthPosition.getVector() ;
        double[] titanCoordinates = titanPosition.getVector() ;

        for( int i = 0 ; i<3 ; i++){
            distanceOfEarthToTitan[i] = titanCoordinates[i] - earthCoordinate[i] ;
        }

        vectorInterface distance =  new Vector(distanceOfEarthToTitan[0] , distanceOfEarthToTitan[1] , distanceOfEarthToTitan[2]) ;
        vectorInterface normalVector = normalize(distance.getVector()) ;

        return normalVector ;
    }


    public vectorInterface getScale(vectorInterface vector , double scale){

        double[] vectorCoordinate = vector.getVector()  ;

        for(int i =  0 ; i<3 ; i++){
            vectorCoordinate[i]*= scale ;
        }

        return new Vector(vectorCoordinate[0] , vectorCoordinate[1] , vectorCoordinate[2]) ;
    }


    public vectorInterface normalize(double[] vector){

        double sum = 0 ;
        for(double value : vector){
            sum+=value ;
        }

        for( int i = 0 ; i<3 ; i++){
            vector[i]/=sum ;
        }

        return new Vector(vector[0] , vector[1] , vector[2]) ;
    }

    /**
     * based on a target position that will determine where it will end up as well as a fuel cap for either cruising
     * the velocity will be the one at  the orbit of earth. Will need an optimizer of its own such that it can find the
     * best trajectory from earth to titan that uses less fuel
     *
     *  @param targetPosition  - the target position of the spaceship
     * @param fuelCap - the max amount of fuel that the spaceship can consume
     */
    public void getStage2(vectorInterface targetPosition , double fuelCap){


    }

    public vectorInterface getTitanPosition( int steps ){

        SolarSystem S = new SolarSystem();
        ArrayList<SpaceObject> solarSystem = S.getSolarSystem();

        for(int t = 0 ; t<steps ; t++){


            solarSystem.get(3).print();

            AccelerationFunction acceleration = new AccelerationFunction();
            VelocityFunction velocity = new VelocityFunction();
            RK4_ODESolver odeSolver = new RK4_ODESolver();

            odeSolver.ComputeODE(3 , S, acceleration ,velocity);

        }

        vectorInterface position = solarSystem.get(8).getPositionVector()   ;

        return new Vector(position.getX() , position.getY() , position.getZ()) ;

    }




}
