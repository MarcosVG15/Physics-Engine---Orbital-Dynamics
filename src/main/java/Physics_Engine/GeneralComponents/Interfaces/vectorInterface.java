package Physics_Engine.GeneralComponents.Interfaces;

import Physics_Engine.GeneralComponents.Vector;

public interface vectorInterface {

    double[] getVector();

    double computeNorm();

    void setVector(vectorInterface v);

    double getX();

    double getY();

    double getZ();



    void print(String type);

    /**
     * Calculates the Euclidean distance between this vector and another vector.
     *
     * @param other The other vector to which the distance is calculated.
     * @return The Euclidean distance between the two vectors.
     */
    double distance(vectorInterface other);

    Vector add(vectorInterface vector2);
    vectorInterface subtract(vectorInterface vector2);
    void scale(double scale);


     void setX(double X);
    void setY(double Y);
    void setZ(double Z);


    }

