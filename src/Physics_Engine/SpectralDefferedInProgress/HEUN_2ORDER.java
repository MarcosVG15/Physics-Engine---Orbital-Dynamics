package src.Physics_Engine.SpectralDefferedInProgress;




import src.Physics_Engine.Interfaces.SpaceObject;
import src.Physics_Engine.Interfaces.function;
import src.Physics_Engine.Interfaces.vectorInterface;

import java.util.ArrayList;

public class HEUN_2ORDER {
    private static final double H = 1;

    public void ComputeODE(double t , SolarSystem solarSystem, function acceleration , function velocity){

        int n = solarSystem.getSolarSystem().size();

        ArrayList<SpaceObject> solarSystemArr = solarSystem.getSolarSystem();

        ArrayList<SpaceObject> snapshotSolarSystem = new ArrayList<>();
        copyArrayList(solarSystemArr ,snapshotSolarSystem);

        vectorInterface[] k1_v = new vectorInterface[n], k1_p = new vectorInterface[n];
        vectorInterface[] k2_v = new vectorInterface[n], k2_p = new vectorInterface[n];


        for( int i = 0 ; i<n ; i++){
            k1_p[i] = scale(solarSystemArr.get(i).getVelocityVector());
            k1_v[i] = scale(acceleration.computeDerivative(i , solarSystemArr.get(i).getPositionVector() , solarSystemArr));
        }

        addSnapshotPosition(snapshotSolarSystem, solarSystemArr ,k1_p , H);
        addSnapshotVelocity(snapshotSolarSystem , solarSystemArr , k1_v , H);

        for ( int i = 0 ; i<n ; i++){
            k2_p[i] = scale(snapshotSolarSystem.get(i).getVelocityVector());
            k2_v[i] = scale(acceleration.computeDerivative(i , snapshotSolarSystem.get(i).getPositionVector() , snapshotSolarSystem));

        }



        for( int i = 0 ; i< n ; i++){
            solarSystemArr.get(i).setPosition(addAll(solarSystemArr.get(i).getPositionVector() , k1_p[i] , k2_p[i] ));
            solarSystemArr.get(i).setVelocity(addAll(solarSystemArr.get(i).getVelocityVector() , k1_v[i] , k2_v[i] ));

        }




    }



    private vectorInterface addAll(vectorInterface currentVector , vectorInterface k1 , vectorInterface k2 ){

        double[] valueHolder = new double[3];

        valueHolder[0] = (currentVector.getX()+ ( H/2.0 * (k1.getX()+ k2.getX() )));
        valueHolder[1] = (currentVector.getY()+ ( H/2.0 * (k1.getY()+ k2.getY() )));
        valueHolder[2] = (currentVector.getZ()+ ( H/2.0 * (k1.getZ()+ k2.getZ())));

        return new Vector(valueHolder[0] , valueHolder[1]  , valueHolder[2]);
    }

    /**
     * Scales the vectors by a H that represents the step size
     * @param vector - the vector that we want to scale
     * @return - returns a scaled vector by H
     */
    private vectorInterface scale(vectorInterface vector){
        double[] valueHolder = new double[3];

        valueHolder[0] = (vector.getX() * H ) ;
        valueHolder[1] = (vector.getY() * H ) ;
        valueHolder[2] = (vector.getZ() * H ) ;

        return new Vector(valueHolder[0] , valueHolder[1]  , valueHolder[2]);
    }


    /**
     * Updates all the vectors for each k that i am analysing ONLY for POSITION
     * @param snapshotArray - the solar system snapshot
     * @param solarSystem - the actual solar system
     * @param currentK = the array of k values
     * @param scalar - the scalar  k that i want to multiple
     */
    private void addSnapshotPosition(ArrayList<SpaceObject> snapshotArray , ArrayList<SpaceObject> solarSystem, vectorInterface[] currentK , double scalar){

        for(int i = 0 ; i < solarSystem.size() ; i++){
            snapshotArray.get(i).setPosition(createSnapshotVector(solarSystem.get(i).getPositionVector() , currentK[i] , scalar));
        }

    }

    /**
     * Updates all the vectors for each k that i am analysing ONLY for VELOCITY
     * @param snapshotArray
     * @param solarSystem
     * @param currentK
     * @param scalar
     */
    private void addSnapshotVelocity(ArrayList<SpaceObject> snapshotArray , ArrayList<SpaceObject> solarSystem, vectorInterface[] currentK , double scalar){

        for(int i = 0 ; i < solarSystem.size() ; i++){
            snapshotArray.get(i).setVelocity(createSnapshotVector(solarSystem.get(i).getVelocityVector() , currentK[i] , scalar));
        }

    }


    /**
     * Creates the RK4 vector for each kn such that i can use it in the { @Link addSnapshotPosition} as well as the velocity one
     *
     * @param currentVector - the current vector position or velocity for each planet
     * @param currentK - the current K that i am working with for each planet
     * @param scalar - the scalar i want to multiply they system with in accordance to the RK4 formulas
     * @return - return an updated vector.
     */
    private vectorInterface createSnapshotVector(vectorInterface currentVector , vectorInterface currentK , double scalar){
        double[] valueHolder = new double[3];

        valueHolder[0] = (currentVector.getX() + currentK.getX() * scalar ) ;
        valueHolder[1] = (currentVector.getY() + currentK.getY() * scalar ) ;
        valueHolder[2] = (currentVector.getZ() + currentK.getZ() * scalar ) ;

        return new Vector(valueHolder[0] , valueHolder[1]  , valueHolder[2]);

    }

    /**
     * Allows me to initialise the astral objects in the snapshot with solar system without passing by reference
     * @param actual - the array we want to copy from
     * @param copy - the array we want the copies to be
     */
    public void copyArrayList(ArrayList<SpaceObject> actual , ArrayList<SpaceObject> copy){

        for( int i = 0  ; i<actual.size() ; i++){
            SpaceObject newObject = new AstralObject(
                    new Vector(actual.get(i).getVelocityVector().getX() , actual.get(i).getVelocityVector().getY(),actual.get(i).getVelocityVector().getZ())
                    , new Vector(actual.get(i).getPositionVector().getX(), actual.get(i).getPositionVector().getY(), actual.get(i).getPositionVector().getZ() )
                    , actual.get(i).getMass());

            copy.add(newObject);
        }
    }
}
