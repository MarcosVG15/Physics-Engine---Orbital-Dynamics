package src.Physics_Engine.SpectralDefferedInProgress;

public interface vectorI {

    double[] getVector();

    double computeNorm();

    void setVector(vectorI v);

    double getX();

    double getY();

    double getZ();

    void print();
}

