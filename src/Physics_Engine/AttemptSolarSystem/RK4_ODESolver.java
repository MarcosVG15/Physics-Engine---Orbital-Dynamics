package src.Physics_Engine.AttemptSolarSystem;



import src.Physics_Engine.AttemptSolarSystem.Interfaces.functionRK4;
import src.Physics_Engine.AttemptSolarSystem.Interfaces.vectorInterfaceRK4;

import java.util.ArrayList;

public class RK4_ODESolver {

    private static final double H = 1;

    public void ComputeODE(double t , SolarSystemRK4 solarSystem, functionRK4 acceleration , functionRK4 velocity){


        int n = solarSystem.getSolarSystem().size();
        ArrayList<AstralObjectRK4> solarSystemArr = solarSystem.getSolarSystem();

        ArrayList<AstralObjectRK4> snapshotSolarSystem = new ArrayList<>();
        copyArrayList(solarSystemArr ,snapshotSolarSystem);

        vectorInterfaceRK4[] k1_v = new vectorInterfaceRK4[n], k1_p = new vectorInterfaceRK4[n];
        vectorInterfaceRK4[] k2_v = new vectorInterfaceRK4[n], k2_p = new vectorInterfaceRK4[n];
        vectorInterfaceRK4[] k3_v = new vectorInterfaceRK4[n], k3_p = new vectorInterfaceRK4[n];
        vectorInterfaceRK4[] k4_v = new vectorInterfaceRK4[n], k4_p = new vectorInterfaceRK4[n];


        for( int i = 0 ; i<n ; i++){
            k1_p[i] = scale(solarSystemArr.get(i).getVelocityVector());
            k1_v[i] = scale(acceleration.computeDerivative(i , solarSystemArr.get(i).getPositionVector() , solarSystemArr));
        }

        addSnapshotPosition(snapshotSolarSystem, solarSystemArr ,k1_p , 0.5);
        addSnapshotVelocity(snapshotSolarSystem , solarSystemArr , k1_v , 0.5);

        for ( int i = 0 ; i<n ; i++){
            k2_p[i] = scale(snapshotSolarSystem.get(i).getVelocityVector());
            k2_v[i] = scale(acceleration.computeDerivative(i , snapshotSolarSystem.get(i).getPositionVector() , snapshotSolarSystem));

        }


        addSnapshotPosition(snapshotSolarSystem, solarSystemArr ,k2_p , 0.5);
        addSnapshotVelocity(snapshotSolarSystem , solarSystemArr , k2_v , 0.5);

        for ( int i = 0 ; i<n ; i++){
            k3_p[i] = scale(snapshotSolarSystem.get(i).getVelocityVector());
            k3_v[i] = scale(acceleration.computeDerivative(i , snapshotSolarSystem.get(i).getPositionVector() , snapshotSolarSystem));

        }

        addSnapshotPosition(snapshotSolarSystem, solarSystemArr ,k3_p , 1);
        addSnapshotVelocity(snapshotSolarSystem , solarSystemArr , k3_v , 1);

        for ( int i = 0 ; i<n ; i++){
            k4_p[i] = scale(snapshotSolarSystem.get(i).getVelocityVector());
            k4_v[i] = scale(acceleration.computeDerivative(i , snapshotSolarSystem.get(i).getPositionVector() , snapshotSolarSystem));

        }

        for( int i = 0 ; i< n ; i++){
            solarSystemArr.get(i).setPosition(addAll(solarSystemArr.get(i).getPositionVector() , k1_p[i] , k2_p[i] , k3_p[i], k4_p[i]));
            solarSystemArr.get(i).setVelocity(addAll(solarSystemArr.get(i).getVelocityVector() , k1_v[i] , k2_v[i] , k3_v[i], k4_v[i]));

        }




    }



    private vectorInterfaceRK4 addAll(vectorInterfaceRK4 currentVector , vectorInterfaceRK4 k1 , vectorInterfaceRK4 k2 , vectorInterfaceRK4 k3 , vectorInterfaceRK4 k4){

        double[] valueHolder = new double[3];

        valueHolder[0] = (currentVector.getX()+ ( 1.0/6.0 * (k1.getX()+ 2* k2.getX() + 2*k3.getX() + k4.getX())));
        valueHolder[1] = (currentVector.getY()+ ( 1.0/6.0 * (k1.getY()+ 2* k2.getY() + 2*k3.getY() + k4.getY())));
        valueHolder[2] = (currentVector.getZ()+ ( 1.0/6.0 * (k1.getZ()+ 2* k2.getZ() + 2*k3.getZ() + k4.getZ())));

        return new VectorRK4(valueHolder[0] , valueHolder[1]  , valueHolder[2]);
    }

    /**
     * Scales the vectors by a H that represents the step size
     * @param vector - the vector that we want to scale
     * @return - returns a scaled vector by H
     */
    private vectorInterfaceRK4 scale(vectorInterfaceRK4 vector){
        double[] valueHolder = new double[3];

        valueHolder[0] = (vector.getX() * H ) ;
        valueHolder[1] = (vector.getY() * H ) ;
        valueHolder[2] = (vector.getZ() * H ) ;

        return new VectorRK4(valueHolder[0] , valueHolder[1]  , valueHolder[2]);
    }


    /**
     * Updates all the vectors for each k that i am analysing ONLY for POSITION
     * @param snapshotArray - the solar system snapshot
     * @param solarSystem - the actual solar system
     * @param currentK = the array of k values
     * @param scalar - the scalar  k that i want to multiple
     */
    private void addSnapshotPosition(ArrayList<AstralObjectRK4> snapshotArray , ArrayList<AstralObjectRK4> solarSystem, vectorInterfaceRK4[] currentK , double scalar){

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
    private void addSnapshotVelocity(ArrayList<AstralObjectRK4> snapshotArray , ArrayList<AstralObjectRK4> solarSystem, vectorInterfaceRK4[] currentK , double scalar){

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
    private vectorInterfaceRK4 createSnapshotVector(vectorInterfaceRK4 currentVector , vectorInterfaceRK4 currentK , double scalar){
        double[] valueHolder = new double[3];

        valueHolder[0] = (currentVector.getX() + currentK.getX() * scalar ) ;
        valueHolder[1] = (currentVector.getY() + currentK.getY() * scalar ) ;
        valueHolder[2] = (currentVector.getZ() + currentK.getZ() * scalar ) ;

        return new VectorRK4(valueHolder[0] , valueHolder[1]  , valueHolder[2]);

    }

    /**
     * Allows me to initialise the astral objects in the snapshot with solar system without passing by reference
     * @param actual - the array we want to copy from
     * @param copy - the array we want the copies to be
     */
    public void copyArrayList(ArrayList<AstralObjectRK4> actual , ArrayList<AstralObjectRK4> copy){

        for( int i = 0  ; i<actual.size() ; i++){
            AstralObjectRK4 newObject = new AstralObjectRK4(
              new VectorRK4(actual.get(i).getVelocityVector().getX() , actual.get(i).getVelocityVector().getY(),actual.get(i).getVelocityVector().getZ())
            , new VectorRK4(actual.get(i).getPositionVector().getX(), actual.get(i).getPositionVector().getY(), actual.get(i).getPositionVector().getZ() )
            , actual.get(i).getMass());

            copy.add(newObject);
        }
    }

}