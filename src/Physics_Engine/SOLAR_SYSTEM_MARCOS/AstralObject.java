package src.Physics_Engine.SOLAR_SYSTEM_MARCOS;

import java.util.ArrayList;

public class AstralObject {
    private Vector velocity ;
    private Vector position ;
    private double Mass;
    private ArrayList<vectorI> positionHistory ;
    private ArrayList<vectorI> velocityHistory ;

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
                new Vector(this.position.getX(),
                        this.position.getY(),
                        this.position.getZ())
        );
        velocityHistory.add(
                new Vector(this.velocity.getX(),
                        this.velocity.getY(),
                        this.velocity.getZ())
        );
    }


    public double getMass(){
        return Mass;
    }

    public void setPosition(vectorI v) {
        position.setVector(v);
        positionHistory.add(
                new Vector(position.getX(), position.getY(), position.getZ())
        );
    }

    public void setVelocity(vectorI v) {
        velocity.setVector(v);
        velocityHistory.add(
                new Vector(velocity.getX(), velocity.getY(), velocity.getZ())
        );
    }



    public vectorI getVelocityVector(){
        return this.velocity;
    }
    public vectorI getPositionVector(){
        return this.position;
    }



    public ArrayList<vectorI> getVelocityLog(){
        return velocityHistory;
    }
    public ArrayList<vectorI> getPositionLog(){
        return positionHistory;
    }


    public void print(){
        position.print();
        velocity.print();
    }



}
