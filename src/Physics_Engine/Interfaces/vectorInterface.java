package src.Physics_Engine.Interfaces;

public interface vectorInterface {

    double[] getVector();

    double computeNorm();

    void setVector(vectorInterface v);

    double getX();

    double getY();

    double getZ();



    void print(String type);
}

