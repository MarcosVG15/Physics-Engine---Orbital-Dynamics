package src.Physics_Engine.RocketMissson;

import src.Physics_Engine.GeneralComponents.AstralObject;
import src.Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import src.Physics_Engine.GeneralComponents.Interfaces.vectorInterface;
import src.Physics_Engine.GeneralComponents.Vector;
import src.Physics_Engine.ProbeMission.ProbeObject;

import java.util.ArrayList;
import java.util.Random;

public class OptimalStepComputation {

    private vectorInterface spaceShipPosition ;
    private vectorInterface titanPosition ;

    public OptimalStepComputation(){

    }

    public void mutate(vectorInterface currentVelocityVector){

        Random random = new Random();
        double[] currentVelocityVectorArray = currentVelocityVector.getVector();

        for(int i = 0 ; i<3 ; i++){
            if(random.nextDouble()*100 <2){
                currentVelocityVectorArray[i] += random.nextDouble()*5 - 2.5 ;
            }
        }
        currentVelocityVector.setVector(new Vector(currentVelocityVectorArray[0] , currentVelocityVectorArray[1] , currentVelocityVectorArray[2]));

    }

    public ArrayList<SpaceObject> getSnapshot(ArrayList<SpaceObject> solarSystem){

        ArrayList<SpaceObject> snapshotSolarSystem = new ArrayList<>();

        for(SpaceObject object : solarSystem){
            Vector VelocityVector = new Vector(object.getVelocityVector().getX()
                    , object.getVelocityVector().getY()
                    , object.getVelocityVector().getZ());

            Vector PositionVector = new Vector(object.getPositionVector().getX()
                    , object.getPositionVector().getY()
                    , object.getPositionVector().getZ());

            if (object instanceof AstralObject) {

                snapshotSolarSystem.add(new AstralObject(VelocityVector, PositionVector, object.getMass()));
                snapshotSolarSystem.get(snapshotSolarSystem.size()-1).setName(object.getName());

            } else if (object instanceof ProbeObject) {

                snapshotSolarSystem.add(new ProbeObject(VelocityVector, PositionVector));
                snapshotSolarSystem.get(snapshotSolarSystem.size()-1).setName(object.getName());

            } else if (object instanceof SpaceShip) {

                snapshotSolarSystem.add(new SpaceShip(VelocityVector, PositionVector));
                snapshotSolarSystem.get(snapshotSolarSystem.size()-1).setName(object.getName());

            } else {
                throw new RuntimeException("Type Not Detected: " + object.getClass().getName());
            }


        }

        return snapshotSolarSystem ;

    }

}
