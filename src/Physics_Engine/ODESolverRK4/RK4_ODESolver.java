package src.Physics_Engine.ODESolverRK4;

import java.util.ArrayList;
import src.Physics_Engine.GeneralComponents.AstralObject;
import src.Physics_Engine.GeneralComponents.Interfaces.SolarSystemInterface;
import src.Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import src.Physics_Engine.GeneralComponents.Interfaces.StateDerivativeFunction;
import src.Physics_Engine.GeneralComponents.Interfaces.function;
import src.Physics_Engine.GeneralComponents.Interfaces.vectorInterface;
import src.Physics_Engine.GeneralComponents.Vector; // Import StateDerivativeFunction
import src.Physics_Engine.RocketMissson.SpaceShip; // Import SpaceShip

public class RK4_ODESolver {

    private static final double H = 60;


    public void ComputeODE(double t , SolarSystemInterface solarSystem, function acceleration , function velocity){


        int n = solarSystem.getSolarSystem().size();
        ArrayList<SpaceObject> solarSystemArr = solarSystem.getSolarSystem();

        ArrayList<SpaceObject> snapshotSolarSystem = new ArrayList<>();
        copyArrayList(solarSystemArr ,snapshotSolarSystem);

        vectorInterface[] k1_v = new vectorInterface[n], k1_p = new vectorInterface[n];
        vectorInterface[] k2_v = new vectorInterface[n], k2_p = new vectorInterface[n];
        vectorInterface[] k3_v = new vectorInterface[n], k3_p = new vectorInterface[n];
        vectorInterface[] k4_v = new vectorInterface[n], k4_p = new vectorInterface[n];


        for( int i = 0 ; i<n ; i++){
            if (i == 0) {
                k1_p[i] = new Vector(0,0,0);
                k1_v[i] = new Vector(0,0,0);;
            }
            k1_p[i] = scale(solarSystemArr.get(i).getVelocityVector());
            k1_v[i] = scale(acceleration.computeDerivative(i , solarSystemArr.get(i).getPositionVector() , solarSystemArr));
        }

        addSnapshotPosition(snapshotSolarSystem, solarSystemArr ,k1_p , 0.5);
        addSnapshotVelocity(snapshotSolarSystem , solarSystemArr , k1_v , 0.5);

        for ( int i = 0 ; i<n ; i++){
            if (i == 0) {
                k2_p[i] = new Vector(0,0,0);
                k2_v[i] = new Vector(0,0,0);;
            }
            k2_p[i] = scale(snapshotSolarSystem.get(i).getVelocityVector());
            k2_v[i] = scale(acceleration.computeDerivative(i , snapshotSolarSystem.get(i).getPositionVector() , snapshotSolarSystem));

        }


        addSnapshotPosition(snapshotSolarSystem, solarSystemArr ,k2_p , 0.5);
        addSnapshotVelocity(snapshotSolarSystem , solarSystemArr , k2_v , 0.5);

        for ( int i = 0 ; i<n ; i++){
            if (i == 0) {
                k3_p[i] = new Vector(0,0,0);
                k3_v[i] = new Vector(0,0,0);;
            }
            k3_p[i] = scale(snapshotSolarSystem.get(i).getVelocityVector());
            k3_v[i] = scale(acceleration.computeDerivative(i , snapshotSolarSystem.get(i).getPositionVector() , snapshotSolarSystem));

        }

        addSnapshotPosition(snapshotSolarSystem, solarSystemArr ,k3_p , 1);
        addSnapshotVelocity(snapshotSolarSystem , solarSystemArr , k3_v , 1);

        for ( int i = 0 ; i<n ; i++){
            if (i == 0) {
                continue;
            }
            k4_p[i] = scale(snapshotSolarSystem.get(i).getVelocityVector());
            k4_v[i] = scale(acceleration.computeDerivative(i , snapshotSolarSystem.get(i).getPositionVector() , snapshotSolarSystem));

        }

        for( int i = 0 ; i< n ; i++){
            if (i == 0) {
                continue;
            }
            solarSystemArr.get(i).setPosition(addAll(solarSystemArr.get(i).getPositionVector() , k1_p[i] , k2_p[i] , k3_p[i], k4_p[i]));
            solarSystemArr.get(i).setVelocity(addAll(solarSystemArr.get(i).getVelocityVector() , k1_v[i] , k2_v[i] , k3_v[i], k4_v[i]));

        }




    }

    /**
     * Computes one step of the RK4 method for a given state vector.
     *
     * @param currentState The current state vector as a double array.
     * @param time The current time.
     * @param dt The time step size.
     * @param derivativeFunction The function that computes the derivative of the state vector.
     * @param params Optional parameters needed for the derivative function.
     * @return The new state vector after one RK4 step.
     */
    public double[] computeODE(double[] currentState, double time, double dt, StateDerivativeFunction derivativeFunction, double[] params) {
        int n = currentState.length;
        double[] k1 = new double[n];
        double[] k2 = new double[n];
        double[] k3 = new double[n];
        double[] k4 = new double[n];
        double[] nextState = new double[n];

        // k1 = dt * f(currentState, time)
        double[] dState1 = derivativeFunction.computeDerivative(currentState, time, params);
        for (int i = 0; i < n; i++) {
            k1[i] = dt * dState1[i];
        }

        // k2 = dt * f(currentState + k1/2, time + dt/2)
        double[] statePlusK1Half = new double[n];
        for (int i = 0; i < n; i++) {
            statePlusK1Half[i] = currentState[i] + k1[i] / 2.0;
        }
        double[] dState2 = derivativeFunction.computeDerivative(statePlusK1Half, time + dt / 2.0, params);
        for (int i = 0; i < n; i++) {
            k2[i] = dt * dState2[i];
        }

        // k3 = dt * f(currentState + k2/2, time + dt/2)
        double[] statePlusK2Half = new double[n];
        for (int i = 0; i < n; i++) {
            statePlusK2Half[i] = currentState[i] + k2[i] / 2.0;
        }
        double[] dState3 = derivativeFunction.computeDerivative(statePlusK2Half, time + dt / 2.0, params);
        for (int i = 0; i < n; i++) {
            k3[i] = dt * dState3[i];
        }

        // k4 = dt * f(currentState + k3, time + dt)
        double[] statePlusK3 = new double[n];
        for (int i = 0; i < n; i++) {
            statePlusK3[i] = currentState[i] + k3[i];
        }
        double[] dState4 = derivativeFunction.computeDerivative(statePlusK3, time + dt, params);
        for (int i = 0; i < n; i++) {
            k4[i] = dt * dState4[i];
        }

        // nextState = currentState + (k1 + 2*k2 + 2*k3 + k4) / 6
        for (int i = 0; i < n; i++) {
            nextState[i] = currentState[i] + (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0;
        }

        return nextState;
    }


    private vectorInterface addAll(vectorInterface currentVector , vectorInterface k1 , vectorInterface k2 , vectorInterface k3 , vectorInterface k4){

        double[] valueHolder = new double[3];

        valueHolder[0] = (currentVector.getX()+ ( 1.0/6.0 * (k1.getX()+ 2* k2.getX() + 2*k3.getX() + k4.getX())));
        valueHolder[1] = (currentVector.getY()+ ( 1.0/6.0 * (k1.getY()+ 2* k2.getY() + 2*k3.getY() + k4.getY())));
        valueHolder[2] = (currentVector.getZ()+ ( 1.0/6.0 * (k1.getZ()+ 2* k2.getZ() + 2*k3.getZ() + k4.getZ())));

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
     * @throws IllegalArgumentException if the actual list contains a SpaceShip object, as it cannot be copied this way.
     */
    public void copyArrayList(ArrayList<SpaceObject> actual , ArrayList<SpaceObject> copy){

        for( int i = 0  ; i<actual.size() ; i++){
             if (actual.get(i) instanceof SpaceShip) {
                // Handle SpaceShip specifically if needed, or throw an exception if it shouldn't be copied here
                throw new IllegalArgumentException("Cannot copy SpaceShip object using this method.");
            }
            SpaceObject newObject = new AstralObject(
              new Vector(actual.get(i).getVelocityVector().getX() , actual.get(i).getVelocityVector().getY(),actual.get(i).getVelocityVector().getZ())
            , new Vector(actual.get(i).getPositionVector().getX(), actual.get(i).getPositionVector().getY(), actual.get(i).getPositionVector().getZ() )
            , actual.get(i).getMass());

            copy.add(newObject);
        }
    }

}
