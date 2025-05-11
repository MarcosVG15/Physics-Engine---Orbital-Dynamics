package src.Physics_Engine.SolarSystem;

import java.util.*;

public class AstralObject {

    /**
     *This class allows us to retrieve the values from Planet
     */

    private double x;
    private double y ;
    private double z;
    private double Vx;
    private double Vy;
    private double Vz;
    private double Ax;
    private double Ay;
    private double Az;



    private double[][] pastCoordinates  ;
    private double[][] pastVelocities   ;

    private double Mass;
    private String name;


    /**
     *
     * @param x - x coordinate
     * @param y - y coordinate
     * @param z - z coordinate
     * @param Vx - velocity int the x direction
     * @param Vy - velocity in the y direction
     * @param Vz - velocity in the z direction
     * @param Mass - the size of the planet of asteroid
     */
    public AstralObject(double x , double y , double z , double Vx, double Vy, double Vz , double Mass, String name){
        this.x = x;
        this.y = y;
        this.z = z;
        this.Vx = Vx;
        this.Vy = Vy;
        this.Vz = Vz;

        this.Mass = Mass;

        this.pastCoordinates =new double[4][3];
        this.pastVelocities = new double[4][3];

        this.name = name;
    }

    public double getMass(){
        return Mass;
    }


    // sets the velocities of the new
    public void setVelocities(double[] newV){
       this.Vx = newV[0];
        this.Vy = newV[1];
        this.Vz = newV[2];
    }


    public double[] getVelocities(){
        return new double[]{Vx, Vy, Vz};
    }


    public void setCoordinates(double[] coordinates){
        this.x = coordinates[0];
        this.y = coordinates[1];
        this.z = coordinates[2];
    }

    public double[] getCoordinates(){
        return new double[]{x, y, z};
    }

    public double[][] getPastCoordinates(){
        return pastCoordinates;
    }

    public double[][] getPastVelocities(){
        return pastVelocities;
    }

    public void setAcceleration(List<AstralObject> solarSystem){
        double Fx = 0;
        double Fy = 0;
        double Fz = 0;
        double Gconstant = 6.67430E-11;

        for(AstralObject object : solarSystem){
            double[] corr = object.getCoordinates();
            double oMass = object.getMass();
            if(x == corr[0]){
                continue;
            }
            double dx = x - corr[0];
            double dy = y - corr[1];
            double dz = z - corr[2];
            double dist = Math.sqrt(dx*dx+dy*dy+dz*dz);

            Fx += Gconstant*oMass*Mass*(dx)/Math.pow(Math.abs(dist), 3);
            Fy += Gconstant*oMass*Mass*(dy)/Math.pow(Math.abs(dist), 3);
            Fz += Gconstant*oMass*Mass*(dz)/Math.pow(Math.abs(dist), 3);
        }
        Fx = -Fx;
        Fy = -Fy;
        Fz = -Fz;

        this.Ax = Fx/Mass;
        this.Ay = Fy/Mass;
        this.Az = Fz/Mass;
    }

    public double[] getAcceleration(){
        return new double[]{Ax, Ay, Az};
    }

    public double[] getAllVariables(){
        return new double[] {x, y, z, Vx, Vy, Vz, Ax, Ay, Az};
    }

    public String getName(){
        return name;
    }
}
