package Physics_Engine.LandingController;

public interface WindModel {

    double getWindX(double altitude, double airspeed);

    double getWindY(double altitude, double airspeed);
}
