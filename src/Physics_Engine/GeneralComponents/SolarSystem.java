package src.Physics_Engine.GeneralComponents;

import src.Physics_Engine.GeneralComponents.Interfaces.SolarSystemInterface;
import src.Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import src.Physics_Engine.GeneralComponents.Interfaces.vectorInterface;

import java.util.ArrayList;

public class SolarSystem implements SolarSystemInterface {
    private final double EARTH_RADIUS = 6370 ;


    private ArrayList<SpaceObject> solarSystem;



    public SolarSystem(){

        SpaceObject sun = new AstralObject(
                new Vector(0, 0, 0),
                new Vector(0, 0, 0),
                1.99E30
        );
        sun.setName("SUN ");

        SpaceObject mercury = new AstralObject(
                new Vector(13.9, -40.3, -4.57),
                new Vector(-5.67E7, -3.23E7, 2.58E6),
                3.30E23
        );
        mercury.setName("mercury");

        SpaceObject venus = new AstralObject(
                new Vector(9.89, -33.7, -1.03),
                new Vector(-1.04E8, -3.19E7, 5.55E6),
                4.87E24
        );
        venus.setName("venus");

        SpaceObject earth = new AstralObject(
                new Vector(5.31, -29.3, 6.69E-4),
                new Vector(-1.47E8, -2.97E7, 2.75E4),
                5.97E24
        );
        earth.setName("earth");

        SpaceObject moon = new AstralObject(
                new Vector(4.53, -28.6, 6.73E-2),
                new Vector(-1.47E8, -2.95E7, 5.29E4),
                7.35E22
        );
        moon.setName("moon");

        SpaceObject mars = new AstralObject(
                new Vector(-11.5, -18.7, -1.11E-1),
                new Vector(-2.15E8, 1.27E8, 7.94E6),
                6.42E23
        );
        mars.setName("mars");

        SpaceObject jupiter = new AstralObject(
                new Vector(-13.2, 12.9, 5.22E-2),
                new Vector(5.54E7, 7.62E8, -4.40E6),
                1.90E27
        );
        jupiter.setName("jupiter");

        SpaceObject saturn = new AstralObject(
                new Vector(0.748, 9.55, -0.196),
                new Vector(1.42E9, -1.91E8, -5.33E7),
                5.68E26
        );
        saturn.setName("saturn");

        SpaceObject titan = new AstralObject(
                new Vector(5.95, 7.68, 0.254),
                new Vector(1.42E9, -1.92E8, -5.28E7),
                1.35E23
        );
        titan.setName("titan");

        SpaceObject uranus = new AstralObject(
                new Vector(-5.72, 3.45, 8.70E-2),
                new Vector(1.62E9, 2.43E9, -1.19E7),
                8.68E25
        );
        uranus.setName("uranus");

        SpaceObject neptune = new AstralObject(
                new Vector(0.0287, 5.47, -0.113),
                new Vector(4.47E9, -5.31E7, -1.02E8),
                1.02E26
        );
        neptune.setName("neptune");


        //--------------------------------------------------------------------------------------------------------------


        // Compute best coordinate to set the probe

        double[] BestCoordinates = getBestInitialPositionCoordinates(titan.getPositionVector().getVector() , earth.getPositionVector().getVector());

        SpaceObject probe = new ProbeObject(
                new Vector(  51.147313,-31.207901,-13.928841),
                new Vector(BestCoordinates[0] , BestCoordinates[1] , BestCoordinates[2])

        );
        probe.setName("PROBE");


        SpaceObject SpaceShip = new SpaceShip(
                new Vector(  5.31, -29.3, 6.69E-4),
                new Vector(BestCoordinates[0]+1 , BestCoordinates[1] +1, BestCoordinates[2]+1 )

        );
        SpaceShip.setName("SpaceShip");

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
        solarSystem.add( neptune);
        solarSystem.add( probe);
        solarSystem.add(SpaceShip);

    }

    public  ArrayList<SpaceObject> getSolarSystem(){
        return solarSystem;
    }

    /**
     * To compute the best initial position for the rocket we compute the vector that is between the earth and titan
     * after which we then scale it down to the radius of the earth by normalising and finally adding it to the vector
     * of earth such that the position remains relative to the sun.
     * @param Titan - coordinates of Titan - start coordinates
     * @param Earth - coordinates of Earth - start coordinates
     * @return
     */
    private double[] getBestInitialPositionCoordinates(double[] Titan , double[] Earth){

        double [] InitialPosition = new double[3] ;
        for( int i = 0 ; i< 3 ; i++){
            InitialPosition[i] = Titan[i] - Earth[i] ;

        }

        // We Normalise scale and add at the same time to save on computation
        double Normal = getModulus(new Vector(InitialPosition[0] , InitialPosition[1], InitialPosition[2]) , new Vector(0,0,0));

        for(int i = 0 ; i<3 ; i++ ){
            InitialPosition[i] = (InitialPosition[i]/Normal) * EARTH_RADIUS + Earth[i];
        }

        return InitialPosition ;
    }
    public double getModulus (vectorInterface vectorProbe , vectorInterface vectorTitan){

        double[] probeValues = vectorProbe.getVector();
        double[] titanValues = vectorTitan.getVector();

        double sum =0 ;

        for(int i = 0 ; i<3 ; i++){
            sum += Math.pow((probeValues[i]- titanValues[i]), 2);
        }

        return Math.sqrt(sum);
    }

    public void setSolarSystem(ArrayList<SpaceObject> replacement){
        solarSystem.clear();

        for (SpaceObject spaceObject : replacement) {
            solarSystem.add(spaceObject.clone()); // make sure each SpaceObject implements clone()
        }
    }

}
