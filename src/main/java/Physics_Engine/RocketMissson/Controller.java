package Physics_Engine.RocketMissson;

import Physics_Engine.GeneralComponents.Interfaces.vectorInterface;

public class Controller {

    public Controller(){

    }

    public void OptimalPath(){

    }



    /**
     * Class that takes in the max fuel is can use and the position coordinate that is limited to the orbit of the earth
     * such that we can optimise this specific part based on the other parameters too .
     *
     * has to start at velocity 0 and uses the thrust based on fuel to move
     *
     * @param targetPosition  - the target position of the spaceship
     * @param startingPosition - starting position of the spaceship
     * @param fuelCap - the max amount of fuel that the spaceship can consume
     */

    public void getStage1(vectorInterface targetPosition , vectorInterface startingPosition , double fuelCap){


    }

    /**
     * based on a target position that will determine where it will end up as well as a fuel cap for either cruising
     * the velocity will be the one at  the orbit of earth. Will need an optimizer of its own such that it can find the
     * best trajectory from earth to titan that uses less fuel
     *
     *  @param targetPosition  - the target position of the spaceship
     * @param fuelCap - the max amount of fuel that the spaceship can consume
     */
    public void getStage2(vectorInterface targetPosition , double fuelCap){


    }




}
