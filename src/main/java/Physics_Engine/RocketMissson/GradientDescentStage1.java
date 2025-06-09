package Physics_Engine.RocketMissson;

import Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import Physics_Engine.GeneralComponents.Interfaces.vectorInterface;
import Physics_Engine.GeneralComponents.SolarSystem;
import Physics_Engine.GeneralComponents.SpaceShip;
import Physics_Engine.GeneralComponents.Thrust;
import Physics_Engine.GeneralComponents.Vector;
import Physics_Engine.ODESolverRK4.AccelerationFunction;
import Physics_Engine.ODESolverRK4.RK4_ODESolver;
import Physics_Engine.ODESolverRK4.VelocityFunction;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import static Physics_Engine.RocketMissson.VARIABLES.STEPSIZE;

public class GradientDescentStage1 {

    private ArrayList<SpaceObject> solarSystem ;
    private Thrust[] ArrayOfThrusts;
    private double TimePenalty ;
    private double VelocityModulus ;
    private vectorInterface targetVelocity ;


    private vectorInterface targetVector ;
    private double fuelCap ;

private  double ALPHA = 1E-3 ;
    private double ALPHA_0 = 1e-1;
    private double beta1 = 0.9;
    private double beta2 = 0.999;
    private double epsilon = 1e-6;
    private double[] m = new double[5];
    private double[] v = new double[5];
    private double t = 0;

    private int amountOfThrust ;

    private double steps ;



    public GradientDescentStage1(ArrayList<SpaceObject> solarSystems , vectorInterface targetVector ,vectorInterface targetVelocity, double fuelCap, int amountOfThrust){
        this.solarSystem = solarSystems ;
        this.targetVector = targetVector ;
        this.fuelCap = fuelCap ;
        this.amountOfThrust = amountOfThrust ;
        this.targetVelocity = targetVelocity ;


        ArrayOfThrusts = new Thrust[amountOfThrust];

//        Random random = new Random();
//
//        // Estimate the required impulse direction based on the target vector
//        double[] targetDirection = targetVector.getVector();
//        double magnitude = getModulus(targetVector);
//        for (int i = 0; i < targetDirection.length; i++) {
//            targetDirection[i] /= magnitude; // Normalize the direction vector
//        }
//
//        // Estimate the required impulse magnitude based on the target velocity
//        double[] targetVelocityDirection = targetVelocity.getVector();
//        double velocityMagnitude = getModulus(targetVelocity);
//         for (int i = 0; i < targetVelocityDirection .length; i++) {
//            targetVelocityDirection [i] /= velocityMagnitude; // Normalize the direction vector
//        }
//
//        double thrustMagnitude = 5000; // Adjust this value as needed
//        double duration = 5; // Adjust this value as needed
//
//        for (int i = 0; i < ArrayOfThrusts.length; i++) {
//            // Calculate thrust components based on the target direction
//            double xThrust = thrustMagnitude * targetDirection[0];
//            double yThrust = thrustMagnitude * targetDirection[1];
//            double zThrust = thrustMagnitude * targetDirection[2];
//
//            Thrust thrust = new Thrust(xThrust, yThrust, zThrust);
//            thrust.setDuration(duration);
//
//            // Distribute start times randomly
//            double startTime = random.nextDouble(100); // Adjust the range as needed
//            thrust.setStartTime(startTime);
//
//            ArrayOfThrusts[i] = thrust;
//
//            System.out.print(Arrays.toString(thrust.getVector()));
//            System.out.println(" Duration : " + thrust.getDuration() + " StartTime : " + thrust.getStartTime());
//        }
        //ArrayOfThrusts[1] = new Thrust(0,0,0);

        ArrayOfThrusts[0] = new Thrust(46.019271053790824, 86.213510920168, 10.596484749385175) ;
        ArrayOfThrusts[0].setDuration(1);
        ArrayOfThrusts[0].setStartTime( 26.621955301890434);

        ArrayOfThrusts[1] = new Thrust(-10.266526601366785, 18.078613951398932, -55.657318973953565) ;
        ArrayOfThrusts[1].setDuration(1);
        ArrayOfThrusts[1].setStartTime(109.8091638751849);

        ArrayOfThrusts[2] = new Thrust(5.830259575697072, 13.576290182302408, -64.03699493723634) ;
        ArrayOfThrusts[2].setDuration(1.3462276957079227);
        ArrayOfThrusts[2].setStartTime(94.65404621287317);




//        NEW COST : 6.335949574098804E7
//        OLD COST : 6.3403174575143434E7
//        GET DISTANCE FOR STOPPING CONDITION ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//                THRUST ARRAY
//        YOU SHOULD SEE AN IMPACT ---[46.019271053790824, 86.213510920168, 10.596484749385175] start Time 26.621955301890434 Duration Time 1.0
//        YOU SHOULD SEE AN IMPACT ---[-10.266526601366785, 18.078613951398932, -55.657318973953565] start Time 109.8091638751849 Duration Time 1.0
//        YOU SHOULD SEE AN IMPACT ---[5.830259575697072, 13.576290182302408, -64.03699493723634] start Time 94.65404621287317 Duration Time 1.3462276957079227
//        Distance  From Earth : [-2.0182191380782433E7, -5.238037445367696E7, -2.9376953421913903E7]
//        END SPACESHIP STATS
//        SpaceShip, Position , 20217513.129959,52386865.228277,29377297.898138
//        SpaceShip  Velocity , -54.070779,12.966621,17.050274
//
//        Distance From Target Velocity : [54.07077863926351, -11.09662149985099, -17.050274098791515]
//        OLD UPDATE OF THRUST *****************************************************************




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

        ArrayList<Double > costs = new ArrayList<>() ;


        boolean hasHit = false ;
        double decayRate = 1e-4;

        double temperature = 10000;
        double temperatureDecayRate = 0.9999;

        double iterations = 0 ;

        while(! hasHit){

            double[][] nextThrustContent = new double[ArrayOfThrusts.length][5] ;

            ALPHA = ALPHA_0 * Math.exp(-decayRate * iterations);

            temperature *= temperatureDecayRate;


            System.out.println("OLD UPDATE OF THRUST *****************************************************************");
            costs.add(getCost(ArrayOfThrusts));

            t++;

            for(int i = 0 ; i<ArrayOfThrusts.length ; i++){

                double[] specificContentOfThrust = ArrayOfThrusts[i].getThrustVector() ;

                Thrust currentThrust = ArrayOfThrusts[i] ;
                for(int j = 0 ; j<specificContentOfThrust.length ; j++){


                    if (j <= 4) {
                        double perturbation = computePerturbation(specificContentOfThrust[j]);
                        Thrust[] TempArray = getPasByValueCopy(ArrayOfThrusts);
                        Thrust updatedThrust = perturb(currentThrust, j, perturbation);

                        TempArray[i] = updatedThrust;

                        double gradient = getSlope(TempArray, costs.get(costs.size()-1), perturbation);

                        System.out.println(" GRADIENT FOR : " + j + " GIVES : " + gradient);

                        m[j] = beta1 * m[j] + (1 - beta1) * gradient;
                        v[j] = beta2 * v[j] + (1 - beta2) * Math.pow(gradient, 2);
                        double m_hat = m[j] / (1 - Math.pow(beta1, t));
                        double v_hat = v[j] / (1 - Math.pow(beta2, t));

                        System.out.println("m_hat: " + m_hat + ", v_hat: " + v_hat);


                        double updated = specificContentOfThrust[j] - ALPHA * m_hat / (Math.sqrt(v_hat) + epsilon) ;
                        if (j == 3) updated = Math.max(0, updated);   // startTime
                        if (j == 4) updated = Math.max(1, updated);


                        nextThrustContent[i][j] = updated ;
                    }
//                    else if(j == 3){
//
//                        double perturbation = computePerturbation(specificContentOfThrust[j]) ;
//
//                        while (perturbation < 0) {
//                            perturbation *= 1.5;
//                        }
//
//                        Thrust[] TempArray = getPasByValueCopy(ArrayOfThrusts);
//                        Thrust updatedThrust = perturb(currentThrust, j, perturbation);
//
//                        TempArray[i] = updatedThrust;
//
//                        double gradient = getSlope(TempArray,  costs.get(costs.size()-1), perturbation);
//
//                        System.out.println(" GRADIENT FOR : " + j + " GIVES : " + gradient);
//
//                        nextThrustContent[i][j] = specificContentOfThrust[j] - 1e-4 * gradient;
//                    }
//                    else if(j == 8){
//
//                        double perturbation = computePerturbation(specificContentOfThrust[j]) ;
//
//                        while (perturbation < 1) {
//                            perturbation *= 1.5;
//                        }
//
//                        Thrust[] TempArray = getPasByValueCopy(ArrayOfThrusts);
//                        Thrust updatedThrust = perturb(currentThrust, j, perturbation);
//
//                        TempArray[i] = updatedThrust;
//
//                        double gradient = getSlope(TempArray, costs.get(costs.size()-1), perturbation);
//
//                        System.out.println(" GRADIENT FOR : " + j + " GIVES : " + gradient);
//
//                        double beta1_duration = 0.7;
//                        double beta2_duration = 0.99;
//                        double ALPHA_duration = 1e-9;
//
//                        m[j] = beta1_duration * m[j] + (1 - beta1_duration) * gradient;
//                        v[j] = beta2_duration * v[j] + (1 - beta2_duration) * Math.pow(gradient, 2);
//                        double m_hat = m[j] / (1 - Math.pow(beta1_duration, t));
//                        double v_hat = v[j] / (1 - Math.pow(beta2_duration, t));
//
//                        nextThrustContent[i][j] = specificContentOfThrust[j] - ALPHA_duration * m_hat / (Math.sqrt(v_hat) + epsilon);
//                    }


                    System.out.println("THE RESULT IS : " + nextThrustContent[i][j]);
                    System.out.println();

                }

            }


            for(int i = 0 ; i<ArrayOfThrusts.length ; i++){


                Thrust newThrust = new Thrust(nextThrustContent[i][0],nextThrustContent[i][1],nextThrustContent[i][2] );
                newThrust.setStartTime(nextThrustContent[i][3]);
                newThrust.setDuration(nextThrustContent[i][4]);

                ArrayOfThrusts[i] = newThrust ;

                costs.add(getCost(ArrayOfThrusts));

                double newCost = costs.get(costs.size()-1) ;
                double costOld = costs.get(costs.size()-2) ;

                System.out.println("NEW COST : " + newCost) ;
                System.out.println("OLD COST : " + costOld) ;

                double delta = newCost - costOld;
                double normalizedDelta = delta / costs.get(0); // Normalize delta by initial cost

                if (delta >0 && random.nextDouble() < Math.exp(normalizedDelta / temperature)) {
                    System.out.println("delta activated");
                    Thrust modded = new Thrust(nextThrustContent[i][0]+random.nextDouble(10)-10*ALPHA
                            ,nextThrustContent[i][1]+random.nextDouble(10)-10*ALPHA
                            ,nextThrustContent[i][2]+random.nextDouble(10)-10*ALPHA );


                    modded.setStartTime(nextThrustContent[i][3]+random.nextDouble(1)*ALPHA);
                    modded.setDuration(nextThrustContent[i][4]+random.nextDouble(1)*ALPHA);

                    ArrayOfThrusts[i] = modded ;



                }

            }


            System.out.println("GET DISTANCE FOR STOPPING CONDITION ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^");
            vectorInterface newDistance = simulate(ArrayOfThrusts);
            hasHit = checkIfDistanceIsZero(newDistance);

            temperature *= temperatureDecayRate;



            iterations++;
        }

        return ArrayOfThrusts ;

    }


    /**
     * It determines the modulus of the two velocities being the target Velocity and the current Velocity such that we
     * can add to the penality for getting into orbit
     * @param currentVelocity - the velocity computed through the simulation
     */
    public void setVelocityPenalty(vectorInterface currentVelocity){

        double[] targetVelocityArray = targetVelocity.getVector() ;
        double[] currentVelocityArray = currentVelocity.getVector() ;

        double[] distanceVector = new double[targetVelocityArray.length] ;
        for(int i = 0 ; i<distanceVector.length ; i++){
            distanceVector[i] = targetVelocityArray[i] - currentVelocityArray[i] ;
        }

        Vector vector = new Vector(distanceVector[0] , distanceVector[1] , distanceVector[2]) ;

        VelocityModulus = getModulus(vector) ;

        System.out.println(" Distance From Target Velocity : "+ Arrays.toString(vector.getVector()));

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
                    thrust.setDuration(20);
                    endTime = startTime+thrust.getDuration() ;
                }
                if (thrust.getStartTime() < 0 ) {
                    System.out.println("problem START TIME");
                    thrust.setStartTime(0);
                    startTime = 0;
                }

               // System.out.println("Start Timer  "+ startTime + " END TIME "+ endTime);

                if(startTime <= steps && endTime>= steps){
                   // System.out.println("YOU SHOULD SEE AN IMPACT ------------------"  + Arrays.toString(thrust.getVector()));

                    SpaceShip spaceShip = (SpaceShip) solarSystemCopy.get(12);

                   vectorInterface VelocityVector = spaceShip.getVelocityVectorPasByValue();
                   VelocityVector.add(scale(thrust , STEPSIZE/spaceShip.getMass()));
                   //System.out.println(" Velocity Updated Vector : " + Arrays.toString(VelocityVector.getVector()));

                    solarSystemCopy.get(12).setVelocity(new Vector(VelocityVector.getX(), VelocityVector.getY(), VelocityVector.getZ() ));
                }

            }
            odeSolver.ComputeODE(3 , solarSystemObject, acceleration ,velocity);

            shouldTerminate = determineStoppingOfSimulation(solarSystemCopy.get(12).getPositionVector());
            //solarSystemObject.getSolarSystem().get(12).print();

            steps++;
            TimePenalty = 31_536_000/STEPSIZE*0.0 ;

            if(steps>31_536_000/STEPSIZE){
                TimePenalty = 31_536_000/STEPSIZE*0.35 ;
                break ;
            }

        }
        vectorInterface endPositionVector   = solarSystemCopy.get(12).getPositionVector() ;


        System.out.println("THRUST ARRAY ");
        for(Thrust thrust :arrayOfThrustUpdate){
            System.out.println("YOU SHOULD SEE AN IMPACT ---"  + Arrays.toString(thrust.getVector()) + " start Time "+ thrust.getStartTime() + " Duration Time " + thrust.getDuration());

        }


        System.out.println("Distance  From Earth : " + Arrays.toString(new Vector(targetVector.getX()-endPositionVector.getX() , targetVector.getY() - endPositionVector.getY() , targetVector.getZ() - endPositionVector.getZ() ).getVector()));
        System.out.println(" END SPACESHIP STATS");
        solarSystemCopy.get(12).print();
        System.out.println(" ");

        setVelocityPenalty(solarSystemCopy.get(12).getVelocityVector());

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
        double penalty = Math.max(0, fuel - fuelCap) * 1e5;
        if(distance<1000){
            return  Math.log(distance/2 + 1) + penalty + VelocityModulus+ TimePenalty ;
        }
        else{
            return  distance + penalty + VelocityModulus+ TimePenalty ;
        }
    }

    /**
     * The amount of change we apply depending on the parameter  we will apply more or less change
     * @param xi - the component we want to perturb
     * @return - the perturbation we have to apply for a given xi
     */
    public double computePerturbation(double xi) {
        double scaled = epsilon * Math.max(1, Math.abs(xi));

        System.out.println(" COMPUTE THE PRETURBATION WITH : "+ xi+ " perturbation : "+ scaled);

        return scaled;
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
            case 3 -> perturbed.setStartTime( t.getStartTime() + h);
            case 4 -> perturbed.setDuration( t.getDuration() + h);

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
            thrustVectorArray[i]*= duration*STEPSIZE;
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
            if(Math.abs(target[i] - projection[i] ) <0.1){
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






}
