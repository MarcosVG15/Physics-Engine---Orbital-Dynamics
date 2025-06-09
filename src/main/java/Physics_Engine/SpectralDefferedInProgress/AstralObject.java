package Physics_Engine.SpectralDefferedInProgress;

import Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import Physics_Engine.GeneralComponents.Interfaces.vectorInterface;

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
    public String getName() {
        return Name;
    }

    @Override
    public boolean hasHitPlanet(SpaceObject astralObject, double Radius) {
        return false;
    }

    @Override
    public SpaceObject clone() {

        AstralObject astralObject = new AstralObject(new Vector(velocity.getX(), velocity.getY(), velocity.getZ())
                , new Vector(position.getX(), position.getY(), position.getZ()), Mass);

        astralObject.setName(Name);
        return astralObject;
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
        return this.velocity;
    }
    public vectorInterface getPositionVector(){
        return this.position;
    }



    public ArrayList<vectorInterface> getVelocityLog(){
        return velocityHistory;
    }
    public ArrayList<vectorInterface> getPositionLog(){
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
