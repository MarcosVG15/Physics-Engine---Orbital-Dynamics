package src.Physics_Engine.RocketMissson;

import src.Physics_Engine.GeneralComponents.AstralObject;
import src.Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import src.Physics_Engine.GeneralComponents.Interfaces.vectorInterface;
import src.Physics_Engine.GeneralComponents.SolarSystem;
import src.Physics_Engine.GeneralComponents.Vector;
import src.Physics_Engine.ODESolverRK4.AccelerationFunction;
import src.Physics_Engine.ODESolverRK4.RK4_ODESolver;
import src.Physics_Engine.ODESolverRK4.VelocityFunction;
import src.Physics_Engine.ProbeMission.ProbeObject;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

public class OptimalStepComputation {

    private ArrayList<SpaceObject> solarSystem ;
    private SolarSystem solarSystemObject ;

    private genomeVector[] population ;


    public OptimalStepComputation(SolarSystem solarSystemObject){

        this.solarSystemObject = solarSystemObject ;
        this.solarSystem = solarSystemObject.getSolarSystem() ;

        this.population = new genomeVector[5000];

        Random random = new Random();

        for(int i = 0 ; i<population.length ; i++){

            double X = random.nextDouble()*400-200 ; // Based on current limitation , the fastest rocket is planned to go 192km/s
            double Y = random.nextDouble()*400-200 ;
            double Z = random.nextDouble()*400-200 ;

            genomeVector vector = new genomeVector(X, Y,Z);
//            System.out.println(X + " " + Y + " " + X);
            population[i] = vector ;

        }

    }



    public void setFitnessValues (genomeVector velocityVector){

        ArrayList<SpaceObject> snapShot = getSnapshot();
        snapShot.get(12).setVelocity(velocityVector);

        //compute the next step such that we can get the future position

        AccelerationFunction acceleration = new AccelerationFunction();
        VelocityFunction velocity = new VelocityFunction();
        RK4_ODESolver odeSolver = new RK4_ODESolver();
        odeSolver.ComputeODE(0 , solarSystemObject, acceleration ,velocity);


        vectorInterface titanPosition = solarSystem.get(8).getPositionVector() ;
        vectorInterface probePosition = solarSystem.get(12).getPositionVector() ;

        velocityVector.setFitness(getModulus(probePosition , titanPosition)) ;

    }


    public double getModulus(vectorInterface position1 , vectorInterface position2){
        double[] position1Array = position1.getVector() ;
        double[] position2Array = position2.getVector() ;

        double sum = 0 ;

        for(int i = 0 ; i<3 ; i++){
            sum+= Math.pow((position2Array[i] - position1Array[i]),2);
        }

        return Math.sqrt(sum);

    }

    /**
     * This method allows me to copy any state of a solar system such that i can populate the genetic algorithm
     * and will allow us to compute the fitness value;
     *
     *
     * @return - It will return copy of the solar system where any changes done to this solar system won't affect the
     *          original one
     */
    public ArrayList<SpaceObject> getSnapshot(){

        ArrayList<SpaceObject> snapshotSolarSystem = new ArrayList<>();

        for(SpaceObject object : solarSystem){
            Vector VelocityVector = new Vector(object.getVelocityVector().getX()
                    , object.getVelocityVector().getY()
                    , object.getVelocityVector().getZ());

            Vector PositionVector = new Vector(object.getPositionVector().getX()
                    , object.getPositionVector().getY()
                    , object.getPositionVector().getZ());

            if (object instanceof AstralObject) {

                snapshotSolarSystem.add(new AstralObject(VelocityVector, PositionVector, object.getMass()));
                snapshotSolarSystem.get(snapshotSolarSystem.size()-1).setName(object.getName());

            } else if (object instanceof ProbeObject) {

                snapshotSolarSystem.add(new ProbeObject(VelocityVector, PositionVector));
                snapshotSolarSystem.get(snapshotSolarSystem.size()-1).setName(object.getName());

            } else if (object instanceof SpaceShip) {

                snapshotSolarSystem.add(new SpaceShip(VelocityVector, PositionVector));
                snapshotSolarSystem.get(snapshotSolarSystem.size()-1).setName(object.getName());

            } else {
                throw new RuntimeException("Type Not Detected: " + object.getClass().getName());
            }


        }

        return snapshotSolarSystem ;

    }

    /**
     * This method is supposed to take to genomes and cross them over . To do this I take the values that correspond to the
     * odd indices of the Vector2 and extract those , for the remaining indicies i extract from vector1 .
     * @param Vector1 - A genome of velocities , essentially just a velocity vector
     * @param Vector2 - Another velocity vector
     * @return - a velocity vector that is a nonrandom combination of vector1 and vector2
     */
    public genomeVector CrossOver(genomeVector Vector1 , genomeVector Vector2){

        double[] Vector1Array = Vector1.getVector() ;
        double[] Vector2Array = Vector2.getVector() ;

        double[] CrossedOver = new double[3];

        CrossedOver[0] = (Vector1.getX() + Vector2.getX()) / 2;
        CrossedOver[1] = (Vector1.getY() + Vector2.getY()) / 2;
        CrossedOver[2] = (Vector1.getZ() + Vector2.getZ()) / 2;


        return new genomeVector(CrossedOver[0] , CrossedOver[1] , CrossedOver[2]);
    }



    public void updatePopulation(){

        // step 5 (external) - update the solar system and repeate


        int selectionGroupSize = 500 ;
        genomeVector[] selectionGroup = new genomeVector[selectionGroupSize];
        int c = 0 ;

        for(int i = population.length-1  ; i>=population.length-selectionGroupSize ; i--){
            selectionGroup[c] = population[i];
            c++;
        }

        Random random = new Random() ;

        for(int i =  0 ; i<population.length ; i++){
            int index1 = biasedIndex(350 , selectionGroupSize,0.5);
            int index2 = biasedIndex(350 , selectionGroupSize,0.5);

            if(index1 == index2){
                i-- ;
                continue;
            }

            population[i] = CrossOver(selectionGroup[index1 ], selectionGroup[index2]);
            population[i].mutate();

        }

 //       System.out.println(Arrays.toString(population[population.length-1].getVector()));
//        System.out.println();
//        for(int i = population.length-1 ; i> population.length-30 ; i--){
//            System.out.println( population[i].getX());
//        }
    }

    /**
     * Returns an integer index that is more likely between 0 and `cutoff`, and
     * exponentially less likely as it increases beyond `cutoff`.
     *
     * @param cutoff     The "preferred" range upper bound (inclusive)
     * @param maxIndex   The maximum index this can return
     * @param decayRate  The exponential decay rate (e.g., 0.5 to 1.5 typical)
     * @return An index in range [0, maxIndex]
     */
    public static int biasedIndex(int cutoff, int maxIndex, double decayRate) {
        Random random = new Random();
        while (true) {
            double r = random.nextDouble();
            double sample = -Math.log(1 - r) / decayRate;

            int index = (int) Math.floor(sample);

            if (index <= cutoff) {
                return index;  // very likely
            } else {
                int extra = (int) Math.floor(sample - cutoff);
                int finalIndex = cutoff + extra;

                if (finalIndex <= maxIndex) {
                    return finalIndex; // exponentially decreasing
                }
                // else: resample
            }
        }
    }





    public genomeVector getBestVectorForStep(){

        boolean foundOptimal = false ;

        for(genomeVector vector : population){
            setFitnessValues(vector);
        }
        // makes the largest value at the top so we will have to extract the bottom 2
        SortingAlgorithm.sort(population);

//        for(genomeVector vector : population){
//            System.out.println(vector.getFitness());
//       }

        genomeVector current = population[population.length-1];
        System.out.println("Vector Content :  "+ Arrays.toString(current.getVector()) + "  Gives this fitness Values :  " + current.getFitness());


        long startTime = System.nanoTime() ;

        while(!foundOptimal){

            updatePopulation();
            for(genomeVector vector : population){
                setFitnessValues(vector);
            }
            // makes the largest value at the top so we will have to extract the bottom 2
            SortingAlgorithm.sort(population);

            genomeVector previous = new genomeVector(current.getX(), current.getY(), current.getZ());
            previous.setFitness(current.getFitness());
            current = population[population.length-1];
            System.out.println("Vector Content :  "+ Arrays.toString(current.getVector()) + "  Gives this fitness Values :  " + current.getFitness());


            if(Compare( previous, current)){
                break ;
            }


        }
        long endTime = System.nanoTime() ;

        double difference = (endTime - startTime)/1_000_000.0 ;
        System.out.println("Duration  :  " + difference);

        return current ;

    }


    public boolean Compare(genomeVector vector1 , genomeVector vector2){
        boolean[] compare = new boolean[3] ;

        double[] vector1Array = vector1.getVector() ;
        double[] vector2Array = vector2.getVector() ;

        for(int i = 0 ; i<3 ; i++){
            if(Math.abs(vector1Array[i] - vector2Array[i])< 0.001){
                compare[i] = true ;
            }
        }

        return compare[0] & compare[1] & compare[2] ;
    }

    // make it such that it is like and average
    public boolean CompareFitnessBased(double previous , double current){
        if(previous>current){
            return true ;
        }
        else{
            return  false ;
        }

    }

    // get best vector
    // run a new step
    //repeat

}
