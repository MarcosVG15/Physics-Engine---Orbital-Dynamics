package src.Physics_Engine.SolarSystem;

public class VelocityDerivative implements ODEFunction{
    public VelocityDerivative(){
    }
    //v = {x, y, z, vx, vy, vz, ax, ay, az}
    @Override
    public double[] computeDerivative(double[] v, double t, double[] a){
        return a;
    }
}
