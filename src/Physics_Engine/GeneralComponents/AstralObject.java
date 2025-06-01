package src.Physics_Engine.GeneralComponents;


import src.Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import src.Physics_Engine.GeneralComponents.Interfaces.vectorInterface;

import java.util.ArrayList;

public class AstralObject implements SpaceObject {
    private Vector velocity ;
    private Vector position ;
    private double Mass;
    private String Name ;
    private ArrayList<vectorInterface> positionHistory ;
    private ArrayList<vectorInterface> velocityHistory ;

    public AstralObject(Vector velocity, Vector position, double Mass) {
        this.Mass     = Mass;
        // make internal copies instead of aliasing
        this.position = new Vector(position.getX(),
                position.getY(),
                position.getZ());
        this.velocity = new Vector(velocity.getX(),
                velocity.getY(),
                velocity.getZ());

        positionHistory = new ArrayList<>();
        velocityHistory = new ArrayList<>();

        // now these snapshots are of your own internal data
        positionHistory.add(
                new Vector(position.getX(),
                        position.getY(),
                        position.getZ())
        );
        velocityHistory.add(
                new Vector(velocity.getX(),
                        velocity.getY(),
                        velocity.getZ())
        );
    }


    public double getMass(){
        return Mass;
    }

    @Override
    public boolean hasHitPlanet(SpaceObject astralObject, double Radius) {
        return false;
    }

    public void setPosition(vectorInterface v) {
        position.setVector(v);
        positionHistory.add(
                new Vector(position.getX(), position.getY(), position.getZ())
        );
    }

    public void setVelocity(vectorInterface v) {
        velocity.setVector(v);
        velocityHistory.add(
                new Vector(velocity.getX(), velocity.getY(), velocity.getZ())
        );
    }



    public vectorInterface getVelocityVector(){
        return velocityHistory.get(velocityHistory.size()-1);
    }
    public vectorInterface getPositionVector(){
        return positionHistory.get(positionHistory.size()-1);
    }



    public ArrayList<vectorInterface> getVelocityLog(){
        return velocityHistory;
    }
    public ArrayList<vectorInterface> getPositionLog(){
        return positionHistory;
    }


    public void print(){
        position.print(Name + ", Position : , ");
        //velocity.print(Name + "  Velocity : ");
    }

    public void setName(String name){
        this.Name = name;
    }





}
