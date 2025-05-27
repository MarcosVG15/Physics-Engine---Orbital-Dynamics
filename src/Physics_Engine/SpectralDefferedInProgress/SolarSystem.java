package src.Physics_Engine.SpectralDefferedInProgress;



import java.util.ArrayList;


public class SolarSystem {

    private static SolarSystem instance;

    private ArrayList<AstralObject> solarSystem;

    public static SolarSystem getInstance() {
        if (instance == null) {
            instance = new SolarSystem();
        }
        return instance;
    }

    private SolarSystem(){

        AstralObject sun = new AstralObject(
                new Vector(0, 0, 0),
                new Vector(0, 0, 0),
                1.99E30
        );
        sun.setName("SUN ");

        AstralObject mercury = new AstralObject(
                new Vector(13.9, -40.3, -4.57),
                new Vector(-5.67E7, -3.23E7, 2.58E6),
                3.30E23
        );
        mercury.setName("mercury");

        AstralObject venus = new AstralObject(
                new Vector(9.89, -33.7, -1.03),
                new Vector(-1.04E8, -3.19E7, 5.55E6),
                4.87E24
        );
        venus.setName("venus");

        AstralObject earth = new AstralObject(
                new Vector(5.31, -29.3, 6.69E-4),
                new Vector(-1.47E8, -2.97E7, 2.75E4),
                5.97E24
        );
        earth.setName("earth");

        AstralObject moon = new AstralObject(
                new Vector(4.53, -28.6, 6.73E-2),
                new Vector(-1.47E8, -2.95E7, 5.29E4),
                7.35E22
        );
        moon.setName("moon");

        AstralObject mars = new AstralObject(
                new Vector(-11.5, -18.7, -1.11E-1),
                new Vector(-2.15E8, 1.27E8, 7.94E6),
                6.42E23
        );
        mars.setName("mars");

        AstralObject jupiter = new AstralObject(
                new Vector(-13.2, 12.9, 5.22E-2),
                new Vector(5.54E7, 7.62E8, -4.40E6),
                1.90E27
        );
        jupiter.setName("jupiter");

        AstralObject saturn = new AstralObject(
                new Vector(0.748, 9.55, -0.196),
                new Vector(1.42E9, -1.91E8, -5.33E7),
                5.68E26
        );
        saturn.setName("saturn");

        AstralObject titan = new AstralObject(
                new Vector(5.95, 7.68, 0.254),
                new Vector(1.42E9, -1.92E8, -5.28E7),
                1.35E23
        );
        titan.setName("titan");

        AstralObject uranus = new AstralObject(
                new Vector(-5.72, 3.45, 8.70E-2),
                new Vector(1.62E9, 2.43E9, -1.19E7),
                8.68E25
        );
        uranus.setName("uranus");

        AstralObject neptune = new AstralObject(
                new Vector(0.0287, 5.47, -0.113),
                new Vector(4.47E9, -5.31E7, -1.02E8),
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

    public ArrayList<AstralObject> getSolarSystem(){
        return solarSystem;
    }

}
