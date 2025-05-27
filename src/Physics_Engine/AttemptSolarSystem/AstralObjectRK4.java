package src.Physics_Engine.AttemptSolarSystem;


import src.Physics_Engine.AttemptSolarSystem.Interfaces.vectorInterfaceRK4;

import java.util.ArrayList;

public class AstralObjectRK4 {
    private VectorRK4 velocity ;
    private VectorRK4 position ;
    private double Mass;
    private String Name ;
    private ArrayList<vectorInterfaceRK4> positionHistory ;
    private ArrayList<vectorInterfaceRK4> velocityHistory ;

    public AstralObjectRK4(VectorRK4 velocity, VectorRK4 position, double Mass) {
        this.Mass     = Mass;
        // make internal copies instead of aliasing
        this.position = new VectorRK4(position.getX(),
                position.getY(),
                position.getZ());
        this.velocity = new VectorRK4(velocity.getX(),
                velocity.getY(),
                velocity.getZ());

        positionHistory = new ArrayList<>();
        velocityHistory = new ArrayList<>();

        // now these snapshots are of your own internal data
        positionHistory.add(
                new VectorRK4(position.getX(),
                        position.getY(),
                        position.getZ())
        );
        velocityHistory.add(
                new VectorRK4(velocity.getX(),
                        velocity.getY(),
                        velocity.getZ())
        );
    }


    public double getMass(){
        return Mass;
    }

    public void setPosition(vectorInterfaceRK4 v) {
        position.setVector(v);
        positionHistory.add(
                new VectorRK4(position.getX(), position.getY(), position.getZ())
        );
    }

    public void setVelocity(vectorInterfaceRK4 v) {
        velocity.setVector(v);
        velocityHistory.add(
                new VectorRK4(velocity.getX(), velocity.getY(), velocity.getZ())
        );
    }



    public vectorInterfaceRK4 getVelocityVector(){
        return this.velocity;
    }
    public vectorInterfaceRK4 getPositionVector(){
        return this.position;
    }



    public ArrayList<vectorInterfaceRK4> getVelocityLog(){
        return velocityHistory;
    }
    public ArrayList<vectorInterfaceRK4> getPositionLog(){
        return positionHistory;
    }


    public void print(){
        position.print(Name + "  Position : ");
        velocity.print(Name + "  Velocity : ");
        System.out.println("");
    }

    public void setName(String name){
        this.Name = name;
    }





}
