package src.Physics_Engine.GeneralComponents;

import src.Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import src.Physics_Engine.GeneralComponents.Interfaces.vectorInterface;

import java.util.ArrayList;

public class SpaceShip extends ProbeObject implements SpaceObject  {
    private double Fuel ;
    public SpaceShip(vectorInterface Velocity , vectorInterface Position){
        super(Velocity,Position);
    }



    public void setFuel(double fuel){
        this.Fuel = fuel ;
    }

    public double getFuel(){
        return Fuel ;
    }



    @Override
    public SpaceObject clone(){

        SpaceShip spaceShip = new SpaceShip(new Vector(super.getVelocityVector().getX() , super.getVelocityVector().getY() , super.getVelocityVector().getZ())
                , new Vector(super.getPositionVector().getX() , super.getPositionVector().getY() , super.getPositionVector().getZ()));
        spaceShip.setName(super.getName());
        spaceShip.setFuel(Fuel);

        return spaceShip ;

    }



}
