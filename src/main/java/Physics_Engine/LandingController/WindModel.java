package src.main.java.Physics_Engine.LandingController;

import java.util.Random;

public class WindModel {
    private Random random;
    private double maxWindForce; // e.g., 0.1 m/s^2

    public WindModel(double maxWindForce) {
        this.random = new Random();
        this.maxWindForce = maxWindForce;
    }

    public double getWindX(double time) {
        // Simple random wind: between -maxWindForce and +maxWindForce
        return (random.nextDouble() * 2 - 1) * maxWindForce;
    }

    public double getWindY(double time) {
        return (random.nextDouble() * 2 - 1) * maxWindForce;
    }

}