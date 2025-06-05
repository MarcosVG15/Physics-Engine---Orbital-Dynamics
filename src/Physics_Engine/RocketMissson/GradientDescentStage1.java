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

import java.sql.SQLOutput;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

public class GradientDescentStage1 {

    private ArrayList<SpaceObject> solarSystem ;
    private Thrust[] ArrayOfThrusts;

    private vectorInterface targetVector ;
    private double fuelCap ;
    private  double ALPHA = 1E-3 ;
    private int amountOfThrust ;

    private double steps ;



    public GradientDescentStage1(ArrayList<SpaceObject> solarSystems , vectorInterface targetVector , double fuelCap, int amountOfThrust){
        this.solarSystem = solarSystems ;
        this.targetVector = targetVector ;
        this.fuelCap = fuelCap ;
        this.amountOfThrust = amountOfThrust ;


        ArrayOfThrusts = new Thrust[amountOfThrust] ;
        Random random = new Random();

        for(int i = 0 ; i<ArrayOfThrusts.length ; i++){

            Thrust thrust = new Thrust(-1*(random.nextInt(50))*100+1, -1*(random.nextInt(50))*100+1,(random.nextInt(50))*100+1 );
            thrust.setDuration(random.nextDouble(5)+10);

            double previous = 0 ;

            if(i != 0 ){
                previous += ArrayOfThrusts[i-1].getDuration() + ArrayOfThrusts[i-1].getStartTime();
            }
            thrust.setStartTime(random.nextDouble(5)+ previous);
            ArrayOfThrusts[i] = thrust ;

            System.out.print(Arrays.toString(thrust.getVector()));
            System.out.println(" Duration : " + thrust.getDuration() + " StartTime : " + thrust.getStartTime());
        }
        //ArrayOfThrusts[1] = new Thrust(0,0,0);


    }


    /**
     * For each thrust and thrust component it applies a small change which will then allow us to determine the slope and
     * thus the modification of that new component. It does this for all thrust components which include the 3 newton forces
     * for each axis as well as the duration and the starting time for each thrust
     *
     */
    public Thrust[] gradientDescent (){
        Random random = new Random() ;
        Thrust[] arrayOfThrustAddition = new Thrust[ArrayOfThrusts.length] ;
        double[] fuelCostPerThrustComponent = new double[ArrayOfThrusts.length] ;


        boolean hasHit = false ;
        double ALPHA_0 = 1;
        double decayRate = 1e-17;

        double iterations = 0 ;

        while(! hasHit){

            double[][] nextThrustContent = new double[ArrayOfThrusts.length][ArrayOfThrusts[0].getThrustVector().length] ;

            ALPHA = ALPHA_0 / (1 + decayRate * iterations);

            if(ALPHA<1e-4){
                ALPHA = 1e-3 ;
            }

            double costOld = getCost(ArrayOfThrusts);

            for(int i = 0 ; i<ArrayOfThrusts.length ; i++){

                double[] specificContentOfThrust = ArrayOfThrusts[i].getThrustVector() ;

                Thrust currentThrust = ArrayOfThrusts[i] ;
                for(int j = 0 ; j<specificContentOfThrust.length ; j++){

                    double perturbation = computePerturbation(specificContentOfThrust[j]) ;
                    Thrust[] TempArray = getPasByValueCopy(ArrayOfThrusts) ;
                    Thrust updatedThrust  = perturb( currentThrust, j ,perturbation ) ;

                    TempArray[i] = updatedThrust ;

                    double gradient = getSlope(TempArray , costOld, perturbation) ;


                    nextThrustContent[i][j] = specificContentOfThrust[j] - ALPHA*gradient ;

                }

            }

            for(int i = 0 ; i<ArrayOfThrusts.length ; i++){
                Thrust newThrust = new Thrust(nextThrustContent[i][0],nextThrustContent[i][1],nextThrustContent[i][2] );
                newThrust.setStartTime(nextThrustContent[i][3]);
                newThrust.setDuration(nextThrustContent[i][4]);

                ArrayOfThrusts[i] = newThrust ;
            }

            vectorInterface newDistance = simulate(ArrayOfThrusts);
            hasHit = checkIfDistanceIsZero(newDistance);



            iterations++;
        }

        return ArrayOfThrusts ;

    }


    /**
     * This method takes in as set of thrust vectors such that i can run the simulation . Indeed the steps indicate the time
     * and thus when the thrust should be initiated and then after simulating untill the projection reaches higher than
     * the actual target then i can terminate and return the distance of the projector to the target. This will allow
     * me to compute the cost function
     * @param arrayOfThrustUpdate - the set of thurst vectors/ components
     * @return - it returns the distance of the rocket to the destination.
     */
    public vectorInterface simulate(Thrust[] arrayOfThrustUpdate){

        boolean shouldTerminate ;
        SolarSystem solarSystemObject = new SolarSystem() ;
        solarSystemObject.setSolarSystem(solarSystem);

        ArrayList<SpaceObject> solarSystemCopy = solarSystemObject.getSolarSystem();

        shouldTerminate = determineStoppingOfSimulation(solarSystemCopy.get(12).getPositionVector());
        System.out.println(" ");

        steps = 0 ;
        while(!shouldTerminate){

            AccelerationFunction acceleration = new AccelerationFunction();
            VelocityFunction velocity = new VelocityFunction();
            RK4_ODESolver odeSolver = new RK4_ODESolver();


            for(Thrust thrust : arrayOfThrustUpdate){

                double startTime  = thrust.getStartTime();
                double endTime = startTime+thrust.getDuration() ;


                if (thrust.getDuration() < 1) {
                    System.out.println("problem");
                    thrust.setDuration(10);
                    endTime = startTime+thrust.getDuration() ;
                }

                if(startTime <= steps && endTime>= steps){
                    System.out.println("YOU SHOULD SEE AN IMPACT -----------------------------------------------------"  + Arrays.toString(thrust.getVector()));

                    SpaceShip spaceShip = (SpaceShip) solarSystemCopy.get(12);

                   vectorInterface VelocityVector = spaceShip.getVelocityVectorPasByValue();
                   VelocityVector.add(scale(thrust , 60/spaceShip.getMass()));
                   //System.out.println(" Velocity Updated Vector : " + Arrays.toString(VelocityVector.getVector()));

                    solarSystemCopy.get(12).setVelocity(new Vector(VelocityVector.getX(), VelocityVector.getY(), VelocityVector.getZ() ));
                }

            }
            odeSolver.ComputeODE(3 , solarSystemObject, acceleration ,velocity);

            shouldTerminate = determineStoppingOfSimulation(solarSystemCopy.get(12).getPositionVector());
            //solarSystemObject.getSolarSystem().get(12).print();

            steps++;
            if(steps>6000){
                break ;
            }

        }
        vectorInterface endPositionVector   = solarSystemCopy.get(12).getPositionVector() ;

        System.out.println("Distance  From Earth : " + Arrays.toString(new Vector(targetVector.getX()-endPositionVector.getX() , targetVector.getY() - endPositionVector.getY() , targetVector.getZ() - endPositionVector.getZ() ).getVector()));
        System.out.println(" END SPACESHIP STATS");
        solarSystemCopy.get(12).print();
        System.out.println(" ");

        return new Vector(targetVector.getX()-endPositionVector.getX() , targetVector.getY() - endPositionVector.getY() , targetVector.getZ() - endPositionVector.getZ() );

    }



    /**
     * Allows me to make a copy of the old Thrust set such that i can modify without
     * changing the values of the previous thrusts
     * @param Array - the array of thrusts that we want to copy
     * @return - returns an array of thrust
     */
    public Thrust[] getPasByValueCopy(Thrust[] Array){

        Thrust[] arrayOfThrustAddition = new Thrust[Array.length] ;

        for(int i = 0 ; i<Array.length ; i++){
            Thrust thrust = Array[i] ;

            arrayOfThrustAddition[i] = new Thrust(thrust.getX() , thrust.getY() , thrust.getZ() ) ;

            arrayOfThrustAddition[i].setStartTime(thrust.getStartTime());
            arrayOfThrustAddition[i].setDuration(thrust.getDuration());
        }

        return arrayOfThrustAddition ;
    }

    /**
     * Gives me the gradient for the gradient descent based on the new and old thrust set
     * @param NewThrustArray  - the updated set of thrust set
     * @param costOld - the previous set of confirmed thruster
     * @param h- the step size/ the perturbation we have applied to the NewThrustArray
     * @return - it returns the slope
     */
    public double getSlope(Thrust[] NewThrustArray, double costOld, double h) {
        double costNew = getCost(NewThrustArray);
        return (costNew - costOld) / h;
    }


    /**
     * Gives the cost of a particular set of thurst components, like a general penality. This will allows us to determine
     * the impact of an individual change on the performance of the system
     * @param thrusts - the set of thursts
     * @return - give the cost / the efficacy of the set of thrust.
     */
    public double getCost(Thrust[] thrusts) {
        vectorInterface distanceVec = simulate(thrusts);
        double distance = getModulus(distanceVec);

        double fuel = 0;
        for (Thrust t : thrusts) {
            fuel += getModulus(getImpulse(t)) / 1000.0;  // kg
        }

        double penalty = Math.max(0, fuel - fuelCap) * 1e4;

        if(distance<10000){
            return distance + penalty +1;
        }
        else{
            return distance + penalty + (100 / (distance + 1));

        }
    }

    /**
     * The amount of change we apply depending on the parameter  we will apply more or less change
     * @param xi - the component we want to perturb
     * @return - the perturbation we have to apply for a given xi
     */
    public double computePerturbation(double xi) {
        double epsilon = 1e-2;
        return epsilon * Math.max(0, Math.abs(xi));
    }


    /**
     * Used for the gradient descent it is essentially the amount of changed applied to a given parameter
     * @param t - this is the thrust component
     * @param index - this allows us to determine what component we want to modify, whether we want to edit the
     *              duration or the x thrust amongst other
     * @param h - is the change that we want to apply
     * @return - returns an updated thrust components with only one modification
     */
    public Thrust perturb(Thrust t, int index, double h) {
        Thrust perturbed = new Thrust(t.getX(), t.getY(), t.getZ());
        perturbed.setStartTime(t.getStartTime());
        perturbed.setDuration(t.getDuration());


        switch (index) {
            case 0 -> perturbed.setX(t.getX() + h);
            case 1 -> perturbed.setY(t.getY() + h);
            case 2 -> perturbed.setZ(t.getZ() + h);
            case 3 -> perturbed.setStartTime(t.getStartTime() + h);
            case 4 -> perturbed.setDuration(t.getDuration() + h);
        }

        return perturbed;
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

    /**
     * Because i assume that the thrusts remain constant the impulse is the thrust times the duration of the thrust
     * @param thrust - the force in newtons that the rocket exerts for a period of time
     * @return - returns the effect of the thrust aka the impulse
     */
    public vectorInterface getImpulse(Thrust thrust){
        double[] thrustVectorArray = thrust.getVector();
        double duration = thrust.getDuration() ;

        for(int i = 0 ; i<thrustVectorArray.length ; i++){
            thrustVectorArray[i]*= duration*60;
        }
        return new Vector(thrustVectorArray[0] , thrustVectorArray[1] , thrustVectorArray[2]);

    }


    public double getSteps(){
        return  steps ;
    }

    private vectorInterface scale(vectorInterface vector , double scale){
        double[] vectorArray = vector.getVector() ;
        double[] scaledArray = new double[vectorArray.length];

        for(int i = 0 ; i<3 ; i++){
            scaledArray[i] = scale*vectorArray[i] ;
        }
        return new Vector(scaledArray[0] , scaledArray[1] , scaledArray[2]);
    }

    /**
     * Returns a boolean value whether the code should keep running or terminate
     */
    private  boolean determineStoppingOfSimulation(vectorInterface projector ){

        boolean[] shouldTerminate = new boolean[3] ;
        double[] projection = getProjection( projector, targetVector).getVector();
        double[] target = targetVector.getVector() ;


        for(int i = 0 ; i<target.length ; i++ ){
            if(Math.abs(target[i] - projection[i] ) <10){
                shouldTerminate[i] = true ;
            }else{
                shouldTerminate[i] = false ;
            }
        }

        return shouldTerminate[0] & shouldTerminate[1] & shouldTerminate[2];
    }


    /**
     * As the name inidcates it returns the project of a vector onto another such that i can figure out when i should terminate the
     * simulation, because if the rocket hasnt reach the target by the time it is at that altitude then there is no point in continuing
     * @param projector - the vector that projects its image onto the target
     * @param target - the projectors target
     * @return - the image / shadow of the projector on  the target
     */
    public vectorInterface getProjection(vectorInterface projector , vectorInterface target){

        vectorInterface projection = new Vector(target.getX() , target.getY() , target.getZ()) ;

        double dotProduct = dotProduct(projector ,  target) ;
        double modulus  = getModulus(target);

        double multiplier = dotProduct/Math.pow(modulus,2) ;
        projection.scale(multiplier);



        return projection ;
    }


    public double dotProduct(vectorInterface vector1 , vectorInterface vector2){

        double[] vector1Array = vector1.getVector() ;
        double[] vector2Array = vector2.getVector() ;

        double sum = 0 ;

        for(int i = 0; i<vector2Array.length ; i++){
            sum+= vector1Array[i]*vector2Array[i] ;
        }

        return sum ;

    }


    /**
     * Essentially it returns the modulus of a vector
     * @param vector - the vector which we want to compute the modulus for
     * @return
     */
    public double getModulus( vectorInterface vector){

        double[] vectorArray = vector.getVector() ;
        double sum = 0 ;

        for(double value : vectorArray){
            sum+= Math.pow(value , 2) ;
        }

        return Math.sqrt(sum);

    }

    public vectorInterface getDistance(){
        return null ;

    }




}
