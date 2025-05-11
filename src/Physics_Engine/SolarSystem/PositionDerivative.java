package src.Physics_Engine.SolarSystem;

public class PositionDerivative implements ODEFunction{

    public PositionDerivative(){
    }

    @Override
    public double[] computeDerivative(double[] x, double t, double[] v) {
        return v;
    }
}