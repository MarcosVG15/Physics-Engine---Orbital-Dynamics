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
import java.util.Random;
import java.util.Collections;
import java.util.Comparator;

public class GeneticAlgorithmOptimizer {

    private final double TITAN_RADIUS = 2575;
    private final int POPULATION_SIZE = 50;
    private final int MAX_GENERATIONS = 100;
    private final double MUTATION_RATE = 0.1;
    private final double CROSSOVER_RATE = 0.8;
    private final double ELITE_RATIO = 0.1;
    
    // Velocity bounds for search space
    private final double MIN_VELOCITY = -100.0;
    private final double MAX_VELOCITY = 100.0;
    
    private Random random;
    private ArrayList<Individual> population;
    private Individual bestSolution;
    
    // Optimization: Cache solar system and reduce simulation time
    private final int SIMULATION_STEPS = (int) (31_536_000 / (STEPSIZE * 10)); // Reduce by factor of 10
    
    public GeneticAlgorithmOptimizer() {
        this.random = new Random();
        this.population = new ArrayList<>();
        initializePopulation();
        evolve();
    }
    
    /**
     * Individual represents a candidate solution (velocity vector)
     */
    private class Individual {
        double[] velocity;
        double fitness;
        boolean fitnessCalculated;
        
        public Individual(double vx, double vy, double vz) {
            this.velocity = new double[]{vx, vy, vz};
            this.fitness = Double.MAX_VALUE;
            this.fitnessCalculated = false;
        }
        
        public Individual(double[] velocity) {
            this.velocity = velocity.clone();
            this.fitness = Double.MAX_VALUE;
            this.fitnessCalculated = false;
        }
        
        public double getFitness() {
            if (!fitnessCalculated) {
                this.fitness = evaluateFitness();
                this.fitnessCalculated = true;
            }
            return fitness;
        }
        
        private double evaluateFitness() {
            return getDistanceToTitan(velocity);
        }
        
        public Individual clone() {
            return new Individual(this.velocity);
        }
    }
    
    /**
     * Initialize population with random velocity vectors
     */
    private void initializePopulation() {
        population.clear();
        
        // Add some known good solutions as seeds
        population.add(new Individual(51.147313,-31.207901,-13.928841));
        population.add(new Individual(45.147313,-35.207901,-7.928841));
        population.add(new Individual(55.147313, -33.207901, -17.928841));
        
        // Fill rest with random individuals
        for (int i = 3; i < POPULATION_SIZE; i++) {
            double vx = MIN_VELOCITY + (MAX_VELOCITY - MIN_VELOCITY) * random.nextDouble();
            double vy = MIN_VELOCITY + (MAX_VELOCITY - MIN_VELOCITY) * random.nextDouble();
            double vz = MIN_VELOCITY + (MAX_VELOCITY - MIN_VELOCITY) * random.nextDouble();
            population.add(new Individual(vx, vy, vz));
        }
    }
    
    /**
     * Main evolution loop
     */
    public void evolve() {
        System.out.println("Starting Genetic Algorithm Optimization...");
        
        for (int generation = 0; generation < MAX_GENERATIONS; generation++) {
            // Evaluate fitness for all individuals
            evaluatePopulation();
            
            // Sort by fitness (ascending - lower is better)
            Collections.sort(population, Comparator.comparingDouble(Individual::getFitness));
            
            // Track best solution
            if (bestSolution == null || population.get(0).getFitness() < bestSolution.getFitness()) {
                bestSolution = population.get(0).clone();
            }
            
            // Print progress
            if (generation % 10 == 0 || generation == MAX_GENERATIONS - 1) {
                System.out.printf("Generation %d: Best fitness = %.2f, Velocity = [%.6f, %.6f, %.6f]\n",
                    generation, bestSolution.getFitness(),
                    bestSolution.velocity[0], bestSolution.velocity[1], bestSolution.velocity[2]);
            }
            
            // Early termination if very close to target
            if (bestSolution.getFitness() < 100.0) {
                System.out.println("Target reached! Stopping early.");
                break;
            }
            
            // Create next generation
            ArrayList<Individual> nextGeneration = createNextGeneration();
            population = nextGeneration;
        }
        
        System.out.println("\nOptimization Complete!");
        System.out.printf("Best Solution: Distance = %.2f\n", bestSolution.getFitness());
        System.out.printf("Optimal Velocity: [%.6f, %.6f, %.6f]\n",
            bestSolution.velocity[0], bestSolution.velocity[1], bestSolution.velocity[2]);
    }
    
    /**
     * Evaluate fitness for entire population
     */
    private void evaluatePopulation() {
        for (Individual individual : population) {
            individual.getFitness(); // This will calculate if not already done
        }
    }
    
    /**
     * Create next generation using selection, crossover, and mutation
     */
    private ArrayList<Individual> createNextGeneration() {
        ArrayList<Individual> nextGen = new ArrayList<>();
        
        // Elitism - keep best individuals
        int eliteCount = (int) (POPULATION_SIZE * ELITE_RATIO);
        for (int i = 0; i < eliteCount; i++) {
            nextGen.add(population.get(i).clone());
        }
        
        // Fill rest with crossover and mutation
        while (nextGen.size() < POPULATION_SIZE) {
            Individual parent1 = tournamentSelection();
            Individual parent2 = tournamentSelection();
            
            Individual child;
            if (random.nextDouble() < CROSSOVER_RATE) {
                child = crossover(parent1, parent2);
            } else {
                child = parent1.clone();
            }
            
            if (random.nextDouble() < MUTATION_RATE) {
                mutate(child);
            }
            
            nextGen.add(child);
        }
        
        return nextGen;
    }
    
    /**
     * Tournament selection
     */
    private Individual tournamentSelection() {
        int tournamentSize = 3;
        Individual best = null;
        
        for (int i = 0; i < tournamentSize; i++) {
            Individual candidate = population.get(random.nextInt(population.size()));
            if (best == null || candidate.getFitness() < best.getFitness()) {
                best = candidate;
            }
        }
        
        return best;
    }
    
    /**
     * Arithmetic crossover
     */
    private Individual crossover(Individual parent1, Individual parent2) {
        double alpha = random.nextDouble();
        double[] childVelocity = new double[3];
        
        for (int i = 0; i < 3; i++) {
            childVelocity[i] = alpha * parent1.velocity[i] + (1 - alpha) * parent2.velocity[i];
        }
        
        return new Individual(childVelocity);
    }
    
    /**
     * Gaussian mutation
     */
    private void mutate(Individual individual) {
        double mutationStrength = 5.0; // Adjust based on problem scale
        
        for (int i = 0; i < 3; i++) {
            if (random.nextDouble() < 0.3) { // 30% chance to mutate each component
                individual.velocity[i] += random.nextGaussian() * mutationStrength;
                
                // Keep within bounds
                individual.velocity[i] = Math.max(MIN_VELOCITY, 
                    Math.min(MAX_VELOCITY, individual.velocity[i]));
            }
        }
        
        // Reset fitness since individual changed
        individual.fitnessCalculated = false;
    }
    
    /**
     * Optimized fitness evaluation - calculates distance to Titan
     */
    private double getDistanceToTitan(double[] velocityArray) {
        try {
            SolarSystem solarSystem = new SolarSystem();
            ArrayList<SpaceObject> solarSystemObjects = solarSystem.getSolarSystem();
            
            // Set probe velocity
            ProbeObject probe = (ProbeObject) solarSystemObjects.get(11);
            probe.setVelocity(new Vector(velocityArray[0], velocityArray[1], velocityArray[2]));
            
            // Run simulation with reduced steps for speed
            AccelerationFunction acceleration = new AccelerationFunction();
            VelocityFunction velocity = new VelocityFunction();
            RK4_ODESolver odeSolver = new RK4_ODESolver();
            
            AstralObject titan = (AstralObject) solarSystemObjects.get(8);
            
            // Check for collision during simulation
            for (int t = 0; t < SIMULATION_STEPS; t++) {
                ProbeObject currentProbe = (ProbeObject) solarSystemObjects.get(11);
                
                if (currentProbe.hasHitPlanet(titan, TITAN_RADIUS)) {
                    return 0.0; // Perfect hit
                }
                
                odeSolver.ComputeODE(0, solarSystem, acceleration, velocity);
                
                // Early termination if probe is getting very far
                double currentDistance = getModulus(currentProbe.getPositionVector(), titan.getPositionVector());
                if (currentDistance > 1e10) { // Very far, likely diverged
                    return currentDistance;
                }
            }
            
            // Return final distance
            ProbeObject finalProbe = (ProbeObject) solarSystemObjects.get(11);
            return getModulus(finalProbe.getPositionVector(), titan.getPositionVector());
            
        } catch (Exception e) {
            // Return large penalty for invalid solutions
            return Double.MAX_VALUE / 2;
        }
    }
    
    /**
     * Calculate Euclidean distance between two position vectors
     */
    private double getModulus(vectorInterface vectorProbe, vectorInterface vectorTitan) {
        double[] probeValues = vectorProbe.getVector();
        double[] titanValues = vectorTitan.getVector();
        
        double sum = 0;
        for (int i = 0; i < 3; i++) {
            sum += Math.pow((probeValues[i] - titanValues[i]), 2);
        }
        
        return Math.sqrt(sum);
    }
    
    /**
     * Get the best solution found
     */
    public double[] getBestVelocity() {
        return bestSolution != null ? bestSolution.velocity.clone() : null;
    }
    
    /**
     * Get the best fitness value
     */
    public double getBestFitness() {
        return bestSolution != null ? bestSolution.getFitness() : Double.MAX_VALUE;
    }
    
    //Main method for testing
//    public static void main(String[] args) {
//        GeneticAlgorithmOptimizer optimizer = new GeneticAlgorithmOptimizer();
//
//        double[] bestVelocity = optimizer.getBestVelocity();
//        if (bestVelocity != null) {
//            System.out.println("\nFinal Results:");
//            System.out.printf("Best Velocity: [%.6f, %.6f, %.6f]\n",
//                bestVelocity[0], bestVelocity[1], bestVelocity[2]);
//            System.out.printf("Final Distance: %.2f\n", optimizer.getBestFitness());
//        }
//    }
}