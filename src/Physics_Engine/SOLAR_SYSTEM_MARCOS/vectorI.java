package src.Physics_Engine.SOLAR_SYSTEM_MARCOS;

public interface vectorI {

    double[] getVector();

    double computeNorm();

    void setVector(vectorI v);

    double getX();

    double getY();

    double getZ();

    void print();
}

