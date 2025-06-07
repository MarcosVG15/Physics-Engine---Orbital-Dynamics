package src.Physics_Engine.GeneralComponents;

public class Thrust extends Vector{

    private double duration ;
    private double startTime ;

    /**
     * @param x - position in x plane
     * @param y - position in y plane
     * @param z - position in z plane
     */
    public Thrust(double x, double y, double z) {
        super(x, y, z);
    }

    public void setDuration(double duration){
        this.duration = duration;
    }
    public double getDuration(){
        return this.duration ;
    }
    public void setStartTime(double startTime){
        this.startTime = startTime ;
    }
    public double getStartTime(){
        return startTime ;
    }

    public double[] getThrustVector(){

        double[] thrustComponents = new double[5];
        thrustComponents[0] = super.getX() ;
        thrustComponents[1] = super.getY() ;
        thrustComponents[2] = super.getZ() ;
        thrustComponents[3] = this.startTime ;
        thrustComponents[4] = this.duration ;

        return thrustComponents ;
    }

    public void print(){

        System.out.println(" THRUST COMPONENTS : "+ super.getX()+ " , "+ super.getY()+ " , "+ super.getZ()+ " Starting : "+ startTime+ " Duration : "+ duration);
    }

}
