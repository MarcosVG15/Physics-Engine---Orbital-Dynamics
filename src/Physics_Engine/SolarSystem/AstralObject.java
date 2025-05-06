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


    private ArrayList<Coordinate> pastCoordinates  ;
    private ArrayList<Velocities> pastVelocities   ;

    private double Mass;


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
    public AstralObject(double x , double y , double z , double Vx, double Vy, double Vz , double Mass){

        Coordinate coordinate  = new Coordinate(x , y, z);
        Velocities velocities = new Velocities(Vx , Vy , Vz);


        this.Mass = Mass;

        this.pastCoordinates =new ArrayList<>() ;
        this.pastVelocities = new ArrayList<>() ;

        pastCoordinates.add(coordinate);
        pastVelocities.add(velocities);

    }

    public double getMass(){
        return Mass;
    }






    // sets the velocities of the new
    public void addVelocities(double[] newV){
       Velocities velocities = new Velocities(newV[0] ,newV[1] , newV[2]);
       pastVelocities.add(velocities);
    }
    public ArrayList<Velocities> getAllVelocities(){
        return pastVelocities;
    }



    public void copyAstralObject(AstralObject other) {
        this.pastCoordinates = other.getAllCoordinates();
        this.pastVelocities = other.getAllVelocities() ;
        this.Mass = other.Mass;
        // Copy other fields if you have them
    }
    public void addCoordinate(double[] coordinates){

        Coordinate coordinate = new Coordinate(coordinates[0] , coordinates[1] , coordinates[2]);
        pastCoordinates.add(coordinate);
    }
    public ArrayList<Coordinate> getAllCoordinates(){

        return pastCoordinates ;
    }
    // returns the last n values of the coordinate array
    public double[][] getSpecificCoordinates(int j ){
        double[][] coordinates = new double[j+1][3];
        int length  = pastCoordinates.size() ;
        for ( int i = length ; i>j ;i--){
            coordinates[length-i] = pastCoordinates.get((int)length-i).getCoordinates() ;
        }

        return coordinates ;
    }
    public double[][] getSpecificVelocities(int j ){
        double[][] velocities = new double[j+1][3];
        int length  = pastVelocities.size() ;
        for ( int i = length ; i>j ;i--){
            velocities[length-i] = pastVelocities.get((int)length-i).getVelocities() ;
        }

        return velocities ;
    }




}
