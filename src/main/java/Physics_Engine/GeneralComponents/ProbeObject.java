package Physics_Engine.GeneralComponents;



import Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import Physics_Engine.GeneralComponents.Interfaces.vectorInterface;

import java.util.ArrayList;

public class ProbeObject implements SpaceObject {

    private vectorInterface velocity ;
    private vectorInterface position ;

    private ArrayList<vectorInterface> positionHistory ;
    private ArrayList<vectorInterface> velocityHistory ;

    private String Name ;


    public ProbeObject(vectorInterface Velocity , vectorInterface Position){
        this.velocity = Velocity;
        this.position = Position ;

        positionHistory = new ArrayList<>();
        velocityHistory = new ArrayList<>();

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

    public void setPosition(vectorInterface v) {
        position.setVector(v);
        positionHistory.add(
                new Vector(position.getX(), position.getY(), position.getZ())
        );
    }

    @Override
    public void setName(String name) {
        this.Name = name;
    }

    @Override
    public void print() {
        position.print(Name + ", Position , ");
        velocity.print(Name + "  Velocity , ");
    }

    @Override
    public double getMass() {
        return 50000 ;
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


    public vectorInterface getVelocityVectorPasByValue(){
        return new Vector(velocityHistory.get(velocityHistory.size()-1).getX() ,velocityHistory.get(velocityHistory.size()-1).getY(),velocityHistory.get(velocityHistory.size()-1).getZ()) ;
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


    public boolean hasHitPlanet(SpaceObject astralObject , double Radius){

        double[] positionArray = position.getVector();
        double[] AstralObjectArray = astralObject.getPositionVector().getVector();
        boolean[] hasHit = new boolean[3];

        for(int i = 0 ; i<positionArray.length ; i++){
            if(Math.abs(positionArray[i] -AstralObjectArray[i]) <= Radius ){
                hasHit[i] = true ;
            }
            else{
                hasHit[i] = false ;
            }
        }

        return hasHit[0] & hasHit[1] & hasHit[2];
    }

    @Override
    public SpaceObject clone() {
        ProbeObject probe = new ProbeObject(new Vector(velocity.getX() , velocity.getY() , velocity.getZ())
                            , new Vector(position.getX() , position.getY() , position.getZ()));
        probe.setName(Name);
        return probe ;
    }

    public String getName(){
        return Name ;
    }

}
