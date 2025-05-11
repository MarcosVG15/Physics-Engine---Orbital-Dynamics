package src.Physics_Engine.SolarSystem;

public interface ODEFunction {
    public double[] computeDerivative(double[] x, double t, double[] v);
}
