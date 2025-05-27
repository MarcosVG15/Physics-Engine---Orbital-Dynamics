package src.Physics_Engine.AttemptSolarSystem.Interfaces;

public interface vectorInterfaceRK4 {

    double[] getVector();

    double computeNorm();

    void setVector(vectorInterfaceRK4 v);

    double getX();

    double getY();

    double getZ();



    void print(String type);
}

