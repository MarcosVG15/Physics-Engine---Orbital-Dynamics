package Physics_Engine.RocketMissson;

import Physics_Engine.GeneralComponents.AstralObject;
import Physics_Engine.GeneralComponents.Interfaces.SpaceObject;
import Physics_Engine.GeneralComponents.SolarSystem;
import Physics_Engine.GeneralComponents.Vector;

import java.util.ArrayList;
import java.util.Arrays;

public class TEST {
    public static void main(String[] args){
        Controller controller = new Controller() ;


        //runStage1() ;
    }

    public static void runStage1(){

        long startTime = System.currentTimeMillis() ;

        SolarSystem solarSystem = new SolarSystem();
//        GradientDescentStage1 gradientDescentStage1 = new GradientDescentStage1(solarSystem.getSolarSystem()
//                , new Vector(-146993666.454047-600,-29700654.885264-600,27287.513994+600 )
//                , 60_000
//                ,1);

       // System.out.println(Arrays.toString(gradientDescentStage1.gradientDescent()));
        long endTime = System.currentTimeMillis() ;

        double timeLapsed = (endTime - startTime)/1000 ;

        System.out.println("Time Taken For Stage1 : "+ timeLapsed);

    }



}
