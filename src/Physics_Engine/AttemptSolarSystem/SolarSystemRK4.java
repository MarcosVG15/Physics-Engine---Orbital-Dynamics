package src.Physics_Engine.AttemptSolarSystem;



import java.util.ArrayList;


public class SolarSystemRK4 {

    private static SolarSystemRK4 instance;

    private ArrayList<AstralObjectRK4> solarSystem;

    public static SolarSystemRK4 getInstance() {
        if (instance == null) {
            instance = new SolarSystemRK4();
        }
        return instance;
    }

    private SolarSystemRK4(){

        AstralObjectRK4 sun = new AstralObjectRK4(
                new VectorRK4(0, 0, 0),
                new VectorRK4(0, 0, 0),
                1.99E30
        );
        sun.setName("SUN ");

        AstralObjectRK4 mercury = new AstralObjectRK4(
                new VectorRK4(13.9, -40.3, -4.57),
                new VectorRK4(-5.67E7, -3.23E7, 2.58E6),
                3.30E23
        );
        mercury.setName("mercury");

        AstralObjectRK4 venus = new AstralObjectRK4(
                new VectorRK4(9.89, -33.7, -1.03),
                new VectorRK4(-1.04E8, -3.19E7, 5.55E6),
                4.87E24
        );
        venus.setName("venus");

        AstralObjectRK4 earth = new AstralObjectRK4(
                new VectorRK4(5.31, -29.3, 6.69E-4),
                new VectorRK4(-1.47E8, -2.97E7, 2.75E4),
                5.97E24
        );
        earth.setName("earth");

        AstralObjectRK4 moon = new AstralObjectRK4(
                new VectorRK4(4.53, -28.6, 6.73E-2),
                new VectorRK4(-1.47E8, -2.95E7, 5.29E4),
                7.35E22
        );
        moon.setName("moon");

        AstralObjectRK4 mars = new AstralObjectRK4(
                new VectorRK4(-11.5, -18.7, -1.11E-1),
                new VectorRK4(-2.15E8, 1.27E8, 7.94E6),
                6.42E23
        );
        mars.setName("mars");

        AstralObjectRK4 jupiter = new AstralObjectRK4(
                new VectorRK4(-13.2, 12.9, 5.22E-2),
                new VectorRK4(5.54E7, 7.62E8, -4.40E6),
                1.90E27
        );
        jupiter.setName("jupiter");

        AstralObjectRK4 saturn = new AstralObjectRK4(
                new VectorRK4(0.748, 9.55, -0.196),
                new VectorRK4(1.42E9, -1.91E8, -5.33E7),
                5.68E26
        );
        saturn.setName("saturn");

        AstralObjectRK4 titan = new AstralObjectRK4(
                new VectorRK4(5.95, 7.68, 0.254),
                new VectorRK4(1.42E9, -1.92E8, -5.28E7),
                1.35E23
        );
        titan.setName("titan");

        AstralObjectRK4 uranus = new AstralObjectRK4(
                new VectorRK4(-5.72, 3.45, 8.70E-2),
                new VectorRK4(1.62E9, 2.43E9, -1.19E7),
                8.68E25
        );
        uranus.setName("uranus");

        AstralObjectRK4 neptune = new AstralObjectRK4(
                new VectorRK4(0.0287, 5.47, -0.113),
                new VectorRK4(4.47E9, -5.31E7, -1.02E8),
                1.02E26
        );
        neptune.setName("neptune");

        solarSystem = new ArrayList<>();

        solarSystem.add( sun);
        solarSystem.add( mercury);
        solarSystem.add( venus);
        solarSystem.add( earth);
        solarSystem.add( moon);
        solarSystem.add( mars);
        solarSystem.add( jupiter);
        solarSystem.add( saturn);
        solarSystem.add( titan);
        solarSystem.add( uranus);
        solarSystem.add(neptune);

    }

    public ArrayList<AstralObjectRK4> getSolarSystem(){
        return solarSystem;
    }

}
