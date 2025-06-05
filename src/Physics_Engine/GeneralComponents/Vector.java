package src.Physics_Engine.GeneralComponents;


import src.Physics_Engine.GeneralComponents.Interfaces.vectorInterface;

/**
 * Allows for opperations and copy of vectors ;
 */
public class Vector implements vectorInterface {

    private double[] values ;

    /**
     *
     * @param x - position in x plane
     * @param y - position in y plane
     * @param z - position in z plane
     */
    public Vector(double x , double  y , double z){
        this.values = new double[]{x, y, z};;

    }

    @Override
    public double[] getVector() {
        return new double[] {values[0] , values[1] , values[2]} ;
    }

    /**
     * computes the norm of the current vector
     * @return - the norm of the vector
     */
    @Override
    public double computeNorm() {
        double sum = 0;

        for (double value : values) {
            sum += Math.pow(value,2);
        }

        return Math.sqrt(sum);
    }

    /**
     * copies the vector without passing by reference
     * @param v - vector that we want to copy
     */
    @Override
    public void setVector(vectorInterface v) {
        values[0] = v.getX();
        values[1] = v.getY();
        values[2] = v.getZ();
    }

    public void print(String  type ){
        System.out.printf(type+ "%f,%f,%f \n"
                , values[0] , values[1], values[2]);
    }

    /**
     * Calculates the Euclidean distance between this vector and another vector.
     *
     * @param other The other vector to which the distance is calculated.
     * @return The Euclidean distance between the two vectors.
     */
    @Override
    public double distance(vectorInterface other) {
        double disX = this.getX() - other.getX();
        double disY = this.getY() - other.getY();
        double disZ = this.getZ() - other.getZ();
        return Math.sqrt(disX * disX + disY * disY + disZ * disZ);
    }

    @Override
    public void add(vectorInterface vector2) {
        double[] vector2Array = vector2.getVector() ;

        for(int i = 0 ; i<vector2Array.length ; i++){
            values[i]+= vector2Array[i] ;
        }


    }

    @Override
    public void scale(double scale) {

        for(int i = 0 ; i<values.length;i++){
            values[i]*= scale ;
        }

    }


    @Override
    public double getX() {
        return values[0];
    }

    @Override
    public double getY() {
        return values[1];
    }

    @Override
    public double getZ() {
        return values[2];
    }


    @Override
    public void setX(double X) {
        values[0] = X;
    }

    @Override
    public void setY(double Y) {
         values[1]= Y;
    }

    @Override
    public void setZ(double Z) {
        values[2] = Z;
    }


}
